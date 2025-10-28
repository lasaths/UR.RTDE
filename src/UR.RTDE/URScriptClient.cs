using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace UR.RTDE
{
    /// <summary>
    /// Minimal TCP client to send URScript to the robot controller.
    /// Typical ports: 30002 (secondary), 30003 (real-time), 50002 (External Control URCap).
    /// </summary>
    public sealed class URScriptClient : IDisposable
    {
        private TcpClient? _client;
        private NetworkStream? _stream;
        private readonly string _host;
        private readonly int _port;
        private bool _disposed;

        public URScriptClient(string host, int port = 30002)
        {
            if (string.IsNullOrWhiteSpace(host))
                throw new ArgumentNullException(nameof(host));
            _host = host;
            _port = port;
        }

        public bool IsConnected => _client?.Connected == true && _stream != null;

        public async Task ConnectAsync(int timeoutMs = 3000, CancellationToken ct = default)
        {
            ThrowIfDisposed();
            if (IsConnected) return;

            var client = new TcpClient();
            using var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
            cts.CancelAfter(timeoutMs);
            var connectTask = client.ConnectAsync(_host, _port);
            using (cts.Token.Register(() => SafeDispose(client))) { await connectTask.ConfigureAwait(false); }

            _client = client;
            _stream = _client.GetStream();
        }

        public async Task SendLineAsync(string scriptLine, CancellationToken ct = default)
        {
            ThrowIfDisposed();
            if (_stream == null) throw new InvalidOperationException("Not connected");
            if (string.IsNullOrWhiteSpace(scriptLine)) return;
            var line = scriptLine.EndsWith("\n") ? scriptLine : scriptLine + "\n";
            var bytes = Encoding.ASCII.GetBytes(line);
            await _stream.WriteAsync(bytes, 0, bytes.Length, ct).ConfigureAwait(false);
            await _stream.FlushAsync(ct).ConfigureAwait(false);
        }

        public async Task SendScriptAsync(string script, CancellationToken ct = default)
        {
            ThrowIfDisposed();
            if (_stream == null) throw new InvalidOperationException("Not connected");
            if (string.IsNullOrWhiteSpace(script)) return;
            var bytes = Encoding.ASCII.GetBytes(script.EndsWith("\n") ? script : script + "\n");
            await _stream.WriteAsync(bytes, 0, bytes.Length, ct).ConfigureAwait(false);
            await _stream.FlushAsync(ct).ConfigureAwait(false);
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            try { _stream?.Dispose(); } catch { }
            try { _client?.Close(); } catch { }
            _stream = null;
            _client = null;
        }

        private static void SafeDispose(TcpClient c)
        {
            try { c.Close(); } catch { }
        }

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(URScriptClient));
        }
    }
}

