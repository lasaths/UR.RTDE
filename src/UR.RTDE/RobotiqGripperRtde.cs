using System;
using System.Threading;
using System.Threading.Tasks;

namespace UR.RTDE
{
    /// <summary>
    /// Robotiq gripper controller using RTDE registers for low-latency commands (Option 3).
    /// Note: This requires small native facade additions to expose RTDE input/output registers.
    /// This class provides the public shape; methods will throw NotSupportedException until the
    /// register API is available.
    /// </summary>
    public sealed class RobotiqGripperRtde
    {
        private readonly RTDEControl _control;
        private readonly RTDEReceive _receive;
        private readonly RTDEIO _io;
        private bool _installed;

        public RobotiqGripperRtde(RTDEControl control, RTDEReceive receive, RTDEIO io)
        {
            _control = control ?? throw new ArgumentNullException(nameof(control));
            _receive = receive ?? throw new ArgumentNullException(nameof(receive));
            _io = io ?? throw new ArgumentNullException(nameof(io));
        }

        // Register map (example; adjust if you already use upper-range registers)
        private const ushort REG_CMD = 0;       // input_int[0]
        private const ushort REG_VAL = 1;       // input_int[1]
        private const ushort REG_STATUS = 0;    // output_int[0]
        private const ushort REG_POS = 1;       // output_int[1]

        // Commands
        private const int CMD_NONE = 0;
        private const int CMD_ACTIVATE = 1;
        private const int CMD_OPEN = 2;
        private const int CMD_CLOSE = 3;
        private const int CMD_MOVE = 4;
        private const int CMD_SET_SPEED = 5;
        private const int CMD_SET_FORCE = 6;

        // Status
        private const int ST_IDLE = 0;
        private const int ST_BUSY = 1;
        private const int ST_ERROR = -1;

        public Task InstallBridgeAsync(CancellationToken ct = default)
        {
            if (_installed) return Task.CompletedTask;
            // URScript bridge: reads input registers and calls Robotiq URCap functions; writes status back
            var script = @" 
def robotiq_rtde_bridge():
  thread bridge_thread():
    while True:
      cmd = read_input_integer_register(%REG_CMD%)
      val = read_input_integer_register(%REG_VAL%)
      if cmd == %CMD_ACTIVATE%:
        rq_activate()
        write_output_integer_register(%REG_STATUS%, %ST_BUSY%)
        sync()
        write_output_integer_register(%REG_STATUS%, %ST_IDLE%)
        write_output_integer_register(%REG_CMD%, %CMD_NONE%)
      elif cmd == %CMD_OPEN%:
        rq_open()
        write_output_integer_register(%REG_STATUS%, %ST_BUSY%)
        sync()
        write_output_integer_register(%REG_STATUS%, %ST_IDLE%)
        write_output_integer_register(%REG_CMD%, %CMD_NONE%)
      elif cmd == %CMD_CLOSE%:
        rq_close()
        write_output_integer_register(%REG_STATUS%, %ST_BUSY%)
        sync()
        write_output_integer_register(%REG_STATUS%, %ST_IDLE%)
        write_output_integer_register(%REG_CMD%, %CMD_NONE%)
      elif cmd == %CMD_MOVE%:
        rq_move(val)
        write_output_integer_register(%REG_STATUS%, %ST_BUSY%)
        sync()
        write_output_integer_register(%REG_STATUS%, %ST_IDLE%)
        write_output_integer_register(%REG_CMD%, %CMD_NONE%)
      elif cmd == %CMD_SET_SPEED%:
        rq_set_speed(val)
        write_output_integer_register(%REG_CMD%, %CMD_NONE%)
      elif cmd == %CMD_SET_FORCE%:
        rq_set_force(val)
        write_output_integer_register(%REG_CMD%, %CMD_NONE%)
      end
      # Optionally publish a position if URCap exposes it via rq_get_pos()
      # write_output_integer_register(%REG_POS%, rq_get_pos())
      sync()
    end
  end
  run bridge_thread()
end
robotiq_rtde_bridge()
";
            // Replace placeholders
            script = script.Replace("%REG_CMD%", REG_CMD.ToString())
                           .Replace("%REG_VAL%", REG_VAL.ToString())
                           .Replace("%REG_STATUS%", REG_STATUS.ToString())
                           .Replace("%REG_POS%", REG_POS.ToString())
                           .Replace("%CMD_NONE%", CMD_NONE.ToString())
                           .Replace("%CMD_ACTIVATE%", CMD_ACTIVATE.ToString())
                           .Replace("%CMD_OPEN%", CMD_OPEN.ToString())
                           .Replace("%CMD_CLOSE%", CMD_CLOSE.ToString())
                           .Replace("%CMD_MOVE%", CMD_MOVE.ToString())
                           .Replace("%CMD_SET_SPEED%", CMD_SET_SPEED.ToString())
                           .Replace("%CMD_SET_FORCE%", CMD_SET_FORCE.ToString())
                           .Replace("%ST_IDLE%", ST_IDLE.ToString())
                           .Replace("%ST_BUSY%", ST_BUSY.ToString())
                           .Replace("%ST_ERROR%", ST_ERROR.ToString());

            _control.SendCustomScript(script);
            _installed = true;
            return Task.CompletedTask;
        }

        public async Task ActivateAsync(CancellationToken ct = default)
        {
            await InstallBridgeAsync(ct).ConfigureAwait(false);
            IssueCommand(CMD_ACTIVATE);
            await WaitIdleAsync(ct).ConfigureAwait(false);
        }

        public async Task OpenAsync(CancellationToken ct = default)
        {
            await InstallBridgeAsync(ct).ConfigureAwait(false);
            IssueCommand(CMD_OPEN);
            await WaitIdleAsync(ct).ConfigureAwait(false);
        }

        public async Task CloseAsync(CancellationToken ct = default)
        {
            await InstallBridgeAsync(ct).ConfigureAwait(false);
            IssueCommand(CMD_CLOSE);
            await WaitIdleAsync(ct).ConfigureAwait(false);
        }

        public async Task MoveAsync(byte position, CancellationToken ct = default)
        {
            await InstallBridgeAsync(ct).ConfigureAwait(false);
            _io.SetInputIntRegister(REG_VAL, position);
            IssueCommand(CMD_MOVE);
            await WaitIdleAsync(ct).ConfigureAwait(false);
        }

        public async Task SetSpeedAsync(byte speed, CancellationToken ct = default)
        {
            await InstallBridgeAsync(ct).ConfigureAwait(false);
            _io.SetInputIntRegister(REG_VAL, speed);
            IssueCommand(CMD_SET_SPEED);
        }

        public async Task SetForceAsync(byte force, CancellationToken ct = default)
        {
            await InstallBridgeAsync(ct).ConfigureAwait(false);
            _io.SetInputIntRegister(REG_VAL, force);
            IssueCommand(CMD_SET_FORCE);
        }

        private void IssueCommand(int cmd)
        {
            _io.SetInputIntRegister(REG_CMD, cmd);
        }

        private async Task WaitIdleAsync(CancellationToken ct)
        {
            var start = DateTime.UtcNow;
            while (!ct.IsCancellationRequested && (DateTime.UtcNow - start).TotalSeconds < 5)
            {
                var st = _receive.GetOutputIntRegister(REG_STATUS);
                if (st == ST_IDLE) return;
                await Task.Delay(20, ct).ConfigureAwait(false);
            }
            throw new TimeoutException("Robotiq operation did not return to IDLE within timeout");
        }
    }
}
