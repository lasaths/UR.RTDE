using System;
using System.Linq;
using UR.RTDE.Native;

namespace UR.RTDE
{
    /// <summary>
    /// Native wrapper around ur_rtde::RobotiqGripper (direct socket driver, no URScript dependency).
    /// Default port: 63352 (Robotiq gripper server on UR controller/URCap).
    /// </summary>
    public sealed class RobotiqGripperNative : IDisposable
    {
        private IntPtr _handle;
        private bool _disposed;

        public RobotiqGripperNative(string hostname, int port = 63352, bool verbose = false)
        {
            if (string.IsNullOrWhiteSpace(hostname))
                throw new ArgumentNullException(nameof(hostname));

            _handle = NativeMethods.ur_rtde_robotiq_gripper_create(hostname, port, verbose);
            if (_handle == IntPtr.Zero)
                throw new RTDEException($"Failed to create Robotiq gripper for {hostname}:{port}");
        }

        public bool IsConnected => !_disposed && NativeMethods.ur_rtde_robotiq_gripper_is_connected(_handle);

        public bool IsActive
        {
            get
            {
                if (_disposed) return false;
                return NativeMethods.ur_rtde_robotiq_gripper_is_active(_handle);
            }
        }

        public float OpenPosition
        {
            get { ThrowIfDisposed(); return NativeMethods.ur_rtde_robotiq_gripper_get_open_position(_handle); }
        }

        public float ClosedPosition
        {
            get { ThrowIfDisposed(); return NativeMethods.ur_rtde_robotiq_gripper_get_closed_position(_handle); }
        }

        public float CurrentPosition
        {
            get { ThrowIfDisposed(); return NativeMethods.ur_rtde_robotiq_gripper_get_current_position(_handle); }
        }

        public bool IsOpen
        {
            get { ThrowIfDisposed(); return NativeMethods.ur_rtde_robotiq_gripper_is_open(_handle); }
        }

        public bool IsClosed
        {
            get { ThrowIfDisposed(); return NativeMethods.ur_rtde_robotiq_gripper_is_closed(_handle); }
        }

        public void Connect(uint timeoutMs = 2000)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_robotiq_gripper_connect(_handle, timeoutMs));
        }

        public void Disconnect()
        {
            if (_disposed) return;
            NativeMethods.ur_rtde_robotiq_gripper_disconnect(_handle);
        }

        public void Activate(bool autoCalibrate = false)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_robotiq_gripper_activate(_handle, autoCalibrate));
        }

        public void AutoCalibrate(float speed = -1f)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_robotiq_gripper_auto_calibrate(_handle, speed));
        }

        public RobotiqObjectStatus Move(float position, float speed = -1f, float force = -1f, RobotiqMoveMode moveMode = RobotiqMoveMode.StartMove)
        {
            ThrowIfDisposed();
            var result = NativeMethods.ur_rtde_robotiq_gripper_move(_handle, position, speed, force, (int)moveMode);
            return ValidateObjectStatus(result, "move");
        }

        public RobotiqObjectStatus Open(float speed = -1f, float force = -1f, RobotiqMoveMode moveMode = RobotiqMoveMode.StartMove)
        {
            ThrowIfDisposed();
            var result = NativeMethods.ur_rtde_robotiq_gripper_open(_handle, speed, force, (int)moveMode);
            return ValidateObjectStatus(result, "open");
        }

        public RobotiqObjectStatus Close(float speed = -1f, float force = -1f, RobotiqMoveMode moveMode = RobotiqMoveMode.StartMove)
        {
            ThrowIfDisposed();
            var result = NativeMethods.ur_rtde_robotiq_gripper_close(_handle, speed, force, (int)moveMode);
            return ValidateObjectStatus(result, "close");
        }

        public void EmergencyRelease(RobotiqPositionId direction, RobotiqMoveMode moveMode = RobotiqMoveMode.WaitFinished)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_robotiq_gripper_emergency_release(_handle, (int)direction, (int)moveMode));
        }

        public RobotiqFaultCode FaultStatus()
        {
            ThrowIfDisposed();
            var code = NativeMethods.ur_rtde_robotiq_gripper_fault_status(_handle);
            return (RobotiqFaultCode)code;
        }

        public void SetUnit(RobotiqMoveParameter parameter, RobotiqUnit unit)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_robotiq_gripper_set_unit(_handle, (int)parameter, (int)unit));
        }

        public void SetPositionRangeMm(int rangeMm)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_robotiq_gripper_set_position_range_mm(_handle, rangeMm));
        }

        public float SetSpeed(float speed)
        {
            ThrowIfDisposed();
            return NativeMethods.ur_rtde_robotiq_gripper_set_speed(_handle, speed);
        }

        public float SetForce(float force)
        {
            ThrowIfDisposed();
            return NativeMethods.ur_rtde_robotiq_gripper_set_force(_handle, force);
        }

        public RobotiqObjectStatus ObjectDetectionStatus()
        {
            ThrowIfDisposed();
            var result = NativeMethods.ur_rtde_robotiq_gripper_object_detection_status(_handle);
            return ValidateObjectStatus(result, "object detection");
        }

        public RobotiqObjectStatus WaitForMotionComplete()
        {
            ThrowIfDisposed();
            var result = NativeMethods.ur_rtde_robotiq_gripper_wait_for_motion_complete(_handle);
            return ValidateObjectStatus(result, "wait");
        }

        public void SetVar(string name, int value)
        {
            ThrowIfDisposed();
            if (string.IsNullOrWhiteSpace(name)) throw new ArgumentNullException(nameof(name));
            CheckStatus(NativeMethods.ur_rtde_robotiq_gripper_set_var(_handle, name, value));
        }

        public int GetVar(string name)
        {
            ThrowIfDisposed();
            if (string.IsNullOrWhiteSpace(name)) throw new ArgumentNullException(nameof(name));
            return NativeMethods.ur_rtde_robotiq_gripper_get_var(_handle, name);
        }

        public int[] GetVars(params string[] names)
        {
            ThrowIfDisposed();
            if (names == null || names.Length == 0) throw new ArgumentNullException(nameof(names));
            var values = new int[names.Length];
            CheckStatus(NativeMethods.ur_rtde_robotiq_gripper_get_vars(_handle, names, (UIntPtr)names.Length, values));
            return values;
        }

        public void SetVars(params (string Name, int Value)[] vars)
        {
            ThrowIfDisposed();
            if (vars == null || vars.Length == 0) throw new ArgumentNullException(nameof(vars));
            var names = vars.Select(v => v.Name).ToArray();
            var values = vars.Select(v => v.Value).ToArray();
            CheckStatus(NativeMethods.ur_rtde_robotiq_gripper_set_vars(_handle, names, values, (UIntPtr)names.Length));
        }

        public void GetNativePositionRange(out int minPosition, out int maxPosition)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_robotiq_gripper_get_native_position_range(_handle, out minPosition, out maxPosition));
        }

        public void SetNativePositionRange(int minPosition, int maxPosition)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_robotiq_gripper_set_native_position_range(_handle, minPosition, maxPosition));
        }

        private void CheckStatus(NativeMethods.Status status)
        {
            if (status == NativeMethods.Status.OK)
                return;

            var errorPtr = NativeMethods.ur_rtde_robotiq_gripper_get_last_error(_handle);
            var errorMsg = NativeMethods.GetString(errorPtr) ?? "Unknown error";
            throw new RTDEException($"Robotiq gripper error ({status}): {errorMsg}");
        }

        private RobotiqObjectStatus ValidateObjectStatus(int code, string operation)
        {
            if (code >= 0)
                return (RobotiqObjectStatus)code;

            var errorPtr = NativeMethods.ur_rtde_robotiq_gripper_get_last_error(_handle);
            var errorMsg = NativeMethods.GetString(errorPtr) ?? "Unknown error";
            throw new RTDEException($"Robotiq gripper {operation} failed: {errorMsg}");
        }

        private void ThrowIfDisposed()
        {
            if (_disposed)
                throw new ObjectDisposedException(nameof(RobotiqGripperNative));
        }

        public void Dispose()
        {
            if (_disposed) return;
            Disconnect();
            if (_handle != IntPtr.Zero)
            {
                NativeMethods.ur_rtde_robotiq_gripper_destroy(_handle);
                _handle = IntPtr.Zero;
            }
            _disposed = true;
        }

        ~RobotiqGripperNative()
        {
            Dispose();
        }
    }

    public enum RobotiqMoveMode
    {
        StartMove = NativeMethods.RobotiqMoveMode.StartMove,
        WaitFinished = NativeMethods.RobotiqMoveMode.WaitFinished
    }

    public enum RobotiqMoveParameter
    {
        Position = NativeMethods.RobotiqMoveParameter.Position,
        Speed = NativeMethods.RobotiqMoveParameter.Speed,
        Force = NativeMethods.RobotiqMoveParameter.Force
    }

    public enum RobotiqUnit
    {
        Device = NativeMethods.RobotiqUnit.Device,
        Normalized = NativeMethods.RobotiqUnit.Normalized,
        Percent = NativeMethods.RobotiqUnit.Percent,
        Millimeter = NativeMethods.RobotiqUnit.Millimeter
    }

    public enum RobotiqPositionId
    {
        Open = NativeMethods.RobotiqPositionId.Open,
        Close = NativeMethods.RobotiqPositionId.Close
    }

    public enum RobotiqObjectStatus
    {
        Moving = 0,
        StoppedOuterObject = 1,
        StoppedInnerObject = 2,
        AtDestination = 3
    }

    public enum RobotiqFaultCode
    {
        NoFault = 0x00,
        FaultActionDelayed = 0x05,
        FaultActivationBit = 0x07,
        FaultTemperature = 0x08,
        FaultComm = 0x09,
        FaultUnderVoltage = 0x0A,
        FaultEmcyReleaseActive = 0x0B,
        FaultInternal = 0x0C,
        FaultActivation = 0x0D,
        FaultOvercurrent = 0x0E,
        FaultEmcyReleaseFinished = 0x0F
    }
}
