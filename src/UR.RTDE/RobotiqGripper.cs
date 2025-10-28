using System;
using System.Threading;
using System.Threading.Tasks;

namespace UR.RTDE
{
    /// <summary>
    /// Robotiq gripper control via URCap URScript functions (Option 2).
    /// Requires Robotiq URCap installed/enabled on the controller.
    /// </summary>
    public sealed class RobotiqGripper : IDisposable
    {
        private readonly URScriptClient _client;
        private bool _disposed;

        /// <summary>
        /// Create a Robotiq gripper controller using URScript over TCP.
        /// </summary>
        /// <param name="host">Robot IP/host</param>
        /// <param name="port">Script port (default 30002)</param>
        public RobotiqGripper(string host, int port = 30002)
        {
            _client = new URScriptClient(host, port);
        }

        public bool IsConnected => _client.IsConnected;

        public Task ConnectAsync(int timeoutMs = 3000, CancellationToken ct = default)
            => _client.ConnectAsync(timeoutMs, ct);

        public async Task ActivateAsync(CancellationToken ct = default)
        {
            ThrowIfDisposed();
            await _client.SendLineAsync("rq_activate()", ct).ConfigureAwait(false);
        }

        public async Task OpenAsync(CancellationToken ct = default)
        {
            ThrowIfDisposed();
            await _client.SendLineAsync("rq_open()", ct).ConfigureAwait(false);
        }

        public async Task CloseAsync(CancellationToken ct = default)
        {
            ThrowIfDisposed();
            await _client.SendLineAsync("rq_close()", ct).ConfigureAwait(false);
        }

        /// <summary>
        /// Move gripper to position (0-255), where 0=open, 255=closed.
        /// </summary>
        public async Task MoveAsync(byte position, CancellationToken ct = default)
        {
            ThrowIfDisposed();
            await _client.SendLineAsync($"rq_move({position})", ct).ConfigureAwait(false);
        }

        /// <summary>
        /// Set speed (0-255). Exact range depends on URCap.
        /// </summary>
        public async Task SetSpeedAsync(byte speed, CancellationToken ct = default)
        {
            ThrowIfDisposed();
            await _client.SendLineAsync($"rq_set_speed({speed})", ct).ConfigureAwait(false);
        }

        /// <summary>
        /// Set force (0-255). Exact range depends on URCap.
        /// </summary>
        public async Task SetForceAsync(byte force, CancellationToken ct = default)
        {
            ThrowIfDisposed();
            await _client.SendLineAsync($"rq_set_force({force})", ct).ConfigureAwait(false);
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            _client.Dispose();
        }

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(RobotiqGripper));
        }
    }
}

