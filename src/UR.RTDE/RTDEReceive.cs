using System;
using System.Threading;
using System.Threading.Tasks;
using UR.RTDE.Native;

namespace UR.RTDE
{
    /// <summary>
    /// Data structure for robot state updates
    /// </summary>
    public sealed class RobotState
    {
        public double[] ActualQ { get; internal set; } = new double[6];
        public double[] ActualQd { get; internal set; } = new double[6];
        public double[] ActualTcpPose { get; internal set; } = new double[6];
        public double[] ActualTcpSpeed { get; internal set; } = new double[6];
        public int RobotMode { get; internal set; }
        public int SafetyMode { get; internal set; }
        public int RuntimeState { get; internal set; }
        public DateTime Timestamp { get; internal set; }
    }

    /// <summary>
    /// RTDE Receive Interface for reading robot state
    /// </summary>
    public sealed class RTDEReceive : IDisposable
    {
        private IntPtr _handle;
        private bool _disposed;
        private readonly string _hostname;
        private CancellationTokenSource? _cts;
        private Task? _receiveTask;

        /// <summary>
        /// Event raised when new robot state is received
        /// </summary>
        public event EventHandler<RobotState>? StateReceived;

        /// <summary>
        /// Create RTDE Receive Interface
        /// </summary>
        /// <param name="hostname">Robot IP address or hostname</param>
        /// <param name="frequency">RTDE frequency in Hz (-1 for default: 500Hz e-Series, 125Hz CB-Series)</param>
        /// <param name="flags">Configuration flags</param>
        public RTDEReceive(string hostname, double frequency = -1.0, ushort flags = 0)
        {
            if (string.IsNullOrWhiteSpace(hostname))
                throw new ArgumentNullException(nameof(hostname));

            _hostname = hostname;
            _handle = NativeMethods.ur_rtde_receive_create(hostname, frequency, flags);

            if (_handle == IntPtr.Zero)
                throw new RTDEConnectionException($"Failed to connect to robot at {hostname}");
        }

        /// <summary>
        /// Check if connected to robot
        /// </summary>
        public bool IsConnected
        {
            get
            {
                ThrowIfDisposed();
                return NativeMethods.ur_rtde_receive_is_connected(_handle);
            }
        }

        /// <summary>
        /// Get actual joint positions
        /// </summary>
        public double[] GetActualQ()
        {
            ThrowIfDisposed();
            var q = new double[6];
            CheckStatus(NativeMethods.ur_rtde_receive_get_actual_q(_handle, q));
            return q;
        }

        /// <summary>
        /// Get actual TCP pose
        /// </summary>
        public double[] GetActualTcpPose()
        {
            ThrowIfDisposed();
            var pose = new double[6];
            CheckStatus(NativeMethods.ur_rtde_receive_get_actual_tcp_pose(_handle, pose));
            return pose;
        }

        /// <summary>
        /// Get actual joint speeds
        /// </summary>
        public double[] GetActualQd()
        {
            ThrowIfDisposed();
            var qd = new double[6];
            CheckStatus(NativeMethods.ur_rtde_receive_get_actual_qd(_handle, qd));
            return qd;
        }

        /// <summary>
        /// Get actual TCP speed
        /// </summary>
        public double[] GetActualTcpSpeed()
        {
            ThrowIfDisposed();
            var speed = new double[6];
            CheckStatus(NativeMethods.ur_rtde_receive_get_actual_tcp_speed(_handle, speed));
            return speed;
        }

        /// <summary>
        /// Get robot mode
        /// </summary>
        public int GetRobotMode()
        {
            ThrowIfDisposed();
            return NativeMethods.ur_rtde_receive_get_robot_mode(_handle);
        }

        /// <summary>
        /// Get safety mode
        /// </summary>
        public int GetSafetyMode()
        {
            ThrowIfDisposed();
            return NativeMethods.ur_rtde_receive_get_safety_mode(_handle);
        }

        /// <summary>
        /// Get runtime state
        /// </summary>
        public int GetRuntimeState()
        {
            ThrowIfDisposed();
            return NativeMethods.ur_rtde_receive_get_runtime_state(_handle);
        }

        /// <summary>
        /// Get standard digital input state
        /// </summary>
        /// <param name="index">Digital input index [0-7]</param>
        public bool GetStandardDigitalIn(int index)
        {
            ThrowIfDisposed();
            if (index < 0 || index > 7)
                throw new ArgumentOutOfRangeException(nameof(index), "Index must be 0-7");
            return NativeMethods.ur_rtde_receive_get_standard_digital_in(_handle, index);
        }

        /// <summary>
        /// Get standard digital output state
        /// </summary>
        /// <param name="index">Digital output index [0-7]</param>
        public bool GetStandardDigitalOut(int index)
        {
            ThrowIfDisposed();
            if (index < 0 || index > 7)
                throw new ArgumentOutOfRangeException(nameof(index), "Index must be 0-7");
            return NativeMethods.ur_rtde_receive_get_standard_digital_out(_handle, index);
        }

        // ====================================================================
        // Extended Data
        // ====================================================================

        /// <summary>
        /// Get target (commanded) joint positions
        /// </summary>
        public double[] GetTargetQ()
        {
            ThrowIfDisposed();
            var q = new double[6];
            CheckStatus(NativeMethods.ur_rtde_receive_get_target_q(_handle, q));
            return q;
        }

        /// <summary>
        /// Get target (commanded) TCP pose
        /// </summary>
        public double[] GetTargetTcpPose()
        {
            ThrowIfDisposed();
            var pose = new double[6];
            CheckStatus(NativeMethods.ur_rtde_receive_get_target_tcp_pose(_handle, pose));
            return pose;
        }

        /// <summary>
        /// Get actual TCP force
        /// </summary>
        public double[] GetActualTcpForce()
        {
            ThrowIfDisposed();
            var force = new double[6];
            CheckStatus(NativeMethods.ur_rtde_receive_get_actual_tcp_force(_handle, force));
            return force;
        }

        /// <summary>
        /// Get joint temperatures
        /// </summary>
        public double[] GetJointTemperatures()
        {
            ThrowIfDisposed();
            var temps = new double[6];
            CheckStatus(NativeMethods.ur_rtde_receive_get_joint_temperatures(_handle, temps));
            return temps;
        }

        /// <summary>
        /// Get actual joint motor currents
        /// </summary>
        public double[] GetActualCurrent()
        {
            ThrowIfDisposed();
            var current = new double[6];
            CheckStatus(NativeMethods.ur_rtde_receive_get_actual_current(_handle, current));
            return current;
        }

        // ====================================================================
        // Safety Status
        // ====================================================================

        /// <summary>
        /// Check if robot is in protective stop
        /// </summary>
        public bool IsProtectiveStopped
        {
            get
            {
                if (_disposed) return false;
                return NativeMethods.ur_rtde_receive_is_protective_stopped(_handle);
            }
        }

        /// <summary>
        /// Check if robot is in emergency stop
        /// </summary>
        public bool IsEmergencyStopped
        {
            get
            {
                if (_disposed) return false;
                return NativeMethods.ur_rtde_receive_is_emergency_stopped(_handle);
            }
        }

        /// <summary>
        /// Get detailed robot status bits
        /// </summary>
        public uint GetRobotStatus()
        {
            ThrowIfDisposed();
            return NativeMethods.ur_rtde_receive_get_robot_status(_handle);
        }

        /// <summary>
        /// Get safety status bits
        /// </summary>
        public uint GetSafetyStatusBits()
        {
            ThrowIfDisposed();
            return NativeMethods.ur_rtde_receive_get_safety_status_bits(_handle);
        }

        // ====================================================================
        // Analog I/O
        // ====================================================================

        /// <summary>
        /// Get standard analog input value
        /// </summary>
        /// <param name="index">Input index (0-1)</param>
        public double GetStandardAnalogInput(int index)
        {
            ThrowIfDisposed();
            if (index < 0 || index > 1)
                throw new ArgumentOutOfRangeException(nameof(index), "Index must be 0 or 1");
            return NativeMethods.ur_rtde_receive_get_standard_analog_input(_handle, (byte)index);
        }

        /// <summary>
        /// Get standard analog output value
        /// </summary>
        /// <param name="index">Output index (0-1)</param>
        public double GetStandardAnalogOutput(int index)
        {
            ThrowIfDisposed();
            if (index < 0 || index > 1)
                throw new ArgumentOutOfRangeException(nameof(index), "Index must be 0 or 1");
            return NativeMethods.ur_rtde_receive_get_standard_analog_output(_handle, (byte)index);
        }

        // ====================================================================
        // Background Receive Thread
        // ====================================================================

        /// <summary>
        /// Start background thread to continuously receive robot state
        /// </summary>
        /// <param name="updateRateMs">Update rate in milliseconds (default: based on RTDE frequency)</param>
        public void StartReceiving(int updateRateMs = 2)
        {
            ThrowIfDisposed();

            if (_receiveTask != null)
                throw new InvalidOperationException("Already receiving. Call StopReceiving first.");

            _cts = new CancellationTokenSource();
            _receiveTask = Task.Run(() => ReceiveLoop(updateRateMs, _cts.Token), _cts.Token);
        }

        /// <summary>
        /// Stop background receive thread
        /// </summary>
        public void StopReceiving()
        {
            if (_cts == null || _receiveTask == null)
                return;

            _cts.Cancel();
            try
            {
                _receiveTask.Wait(5000);
            }
            catch (AggregateException) { }
            finally
            {
                _cts?.Dispose();
                _cts = null;
                _receiveTask = null;
            }
        }

        private void ReceiveLoop(int updateRateMs, CancellationToken token)
        {
            var state = new RobotState();

            while (!token.IsCancellationRequested && IsConnected)
            {
                try
                {
                    state.ActualQ = GetActualQ();
                    state.ActualQd = GetActualQd();
                    state.ActualTcpPose = GetActualTcpPose();
                    state.ActualTcpSpeed = GetActualTcpSpeed();
                    state.RobotMode = GetRobotMode();
                    state.SafetyMode = GetSafetyMode();
                    state.RuntimeState = GetRuntimeState();
                    state.Timestamp = DateTime.UtcNow;

                    StateReceived?.Invoke(this, state);

                    if (updateRateMs > 0)
                        Thread.Sleep(updateRateMs);
                }
                catch (Exception ex) when (!(ex is ObjectDisposedException))
                {
                    // Log or handle errors (connection lost, etc.)
                    break;
                }
            }
        }

        private void CheckStatus(NativeMethods.Status status)
        {
            if (status == NativeMethods.Status.OK)
                return;

            var errorPtr = NativeMethods.ur_rtde_receive_get_last_error(_handle);
            var errorMsg = NativeMethods.GetString(errorPtr) ?? "Unknown error";

            throw status switch
            {
                NativeMethods.Status.ErrorConnection => new RTDEConnectionException($"Connection error: {errorMsg}"),
                NativeMethods.Status.ErrorTimeout => new RTDEException($"Timeout: {errorMsg}"),
                NativeMethods.Status.ErrorNotConnected => new RTDEConnectionException($"Not connected: {errorMsg}"),
                _ => new RTDEException($"Error {status}: {errorMsg}")
            };
        }

        private void ThrowIfDisposed()
        {
            if (_disposed)
                throw new ObjectDisposedException(nameof(RTDEReceive));
        }

        public void Dispose()
        {
            if (_disposed)
                return;

            StopReceiving();

            if (_handle != IntPtr.Zero)
            {
                NativeMethods.ur_rtde_receive_destroy(_handle);
                _handle = IntPtr.Zero;
            }

            _disposed = true;
        }

        ~RTDEReceive()
        {
            Dispose();
        }
    }
}
