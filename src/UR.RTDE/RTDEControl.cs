using System;
using System.Runtime.InteropServices;
using UR.RTDE.Native;

namespace UR.RTDE
{
    /// <summary>
    /// RTDE Control Interface for commanding the robot
    /// </summary>
    public sealed class RTDEControl : IDisposable
    {
        private IntPtr _handle;
        private bool _disposed;
        private readonly string _hostname;

        /// <summary>
        /// Create RTDE Control Interface
        /// </summary>
        /// <param name="hostname">Robot IP address or hostname</param>
        /// <param name="frequency">RTDE frequency in Hz (-1 for default: 500Hz e-Series, 125Hz CB-Series)</param>
        /// <param name="flags">Configuration flags</param>
        /// <param name="urCapPort">External URCap port (default: 50002)</param>
        public RTDEControl(string hostname, double frequency = -1.0, ushort flags = (ushort)NativeMethods.Flags.Default, int urCapPort = 50002)
        {
            if (string.IsNullOrWhiteSpace(hostname))
                throw new ArgumentNullException(nameof(hostname));

            _hostname = hostname;
            _handle = NativeMethods.ur_rtde_control_create(hostname, frequency, flags, urCapPort);

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
                return NativeMethods.ur_rtde_control_is_connected(_handle);
            }
        }

        /// <summary>
        /// Disconnect from robot
        /// </summary>
        public void Disconnect()
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_disconnect(_handle));
        }

        /// <summary>
        /// Reconnect to robot after lost connection
        /// </summary>
        public void Reconnect()
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_reconnect(_handle));
        }

        /// <summary>
        /// Move to joint position (linear in joint-space)
        /// </summary>
        /// <param name="q">Joint positions [6] in radians</param>
        /// <param name="speed">Joint speed of leading axis [rad/s]</param>
        /// <param name="acceleration">Joint acceleration [rad/s^2]</param>
        /// <param name="asynchronous">Non-blocking if true</param>
        public void MoveJ(double[] q, double speed = 1.05, double acceleration = 1.4, bool asynchronous = false)
        {
            ThrowIfDisposed();
            ValidateArray(q, 6, nameof(q));
            CheckStatus(NativeMethods.ur_rtde_control_move_j(_handle, q, speed, acceleration, asynchronous));
        }

        /// <summary>
        /// Move to pose (linear in tool-space)
        /// </summary>
        /// <param name="pose">Target pose [x, y, z, rx, ry, rz] (axis-angle)</param>
        /// <param name="speed">Tool speed [m/s]</param>
        /// <param name="acceleration">Tool acceleration [m/s^2]</param>
        /// <param name="asynchronous">Non-blocking if true</param>
        public void MoveL(double[] pose, double speed = 0.25, double acceleration = 1.2, bool asynchronous = false)
        {
            ThrowIfDisposed();
            ValidateArray(pose, 6, nameof(pose));
            CheckStatus(NativeMethods.ur_rtde_control_move_l(_handle, pose, speed, acceleration, asynchronous));
        }

        /// <summary>
        /// Stop robot (linear in joint-space)
        /// </summary>
        /// <param name="acceleration">Deceleration [rad/s^2]</param>
        /// <param name="asynchronous">Non-blocking if true</param>
        public void StopJ(double acceleration = 2.0, bool asynchronous = false)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_stop_j(_handle, acceleration, asynchronous));
        }

        /// <summary>
        /// Stop robot (linear in tool-space)
        /// </summary>
        /// <param name="acceleration">Deceleration [m/s^2]</param>
        /// <param name="asynchronous">Non-blocking if true</param>
        public void StopL(double acceleration = 10.0, bool asynchronous = false)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_stop_l(_handle, acceleration, asynchronous));
        }

        /// <summary>
        /// Joint speed control
        /// </summary>
        /// <param name="qd">Joint speeds [6] in rad/s</param>
        /// <param name="acceleration">Joint acceleration [rad/s^2]</param>
        /// <param name="time">Duration [s] (0 = indefinite)</param>
        public void SpeedJ(double[] qd, double acceleration = 0.5, double time = 0.0)
        {
            ThrowIfDisposed();
            ValidateArray(qd, 6, nameof(qd));
            CheckStatus(NativeMethods.ur_rtde_control_speed_j(_handle, qd, acceleration, time));
        }

        /// <summary>
        /// Tool speed control
        /// </summary>
        /// <param name="xd">Tool speed [6] in m/s and rad/s</param>
        /// <param name="acceleration">Tool acceleration [m/s^2]</param>
        /// <param name="time">Duration [s] (0 = indefinite)</param>
        public void SpeedL(double[] xd, double acceleration = 0.25, double time = 0.0)
        {
            ThrowIfDisposed();
            ValidateArray(xd, 6, nameof(xd));
            CheckStatus(NativeMethods.ur_rtde_control_speed_l(_handle, xd, acceleration, time));
        }

        /// <summary>
        /// Servo to joint position
        /// </summary>
        /// <param name="q">Joint positions [6] in radians</param>
        /// <param name="speed">NOT used</param>
        /// <param name="acceleration">NOT used</param>
        /// <param name="time">Command duration [s]</param>
        /// <param name="lookaheadTime">Smoothing [0.03-0.2 s]</param>
        /// <param name="gain">Proportional gain [100-2000]</param>
        public void ServoJ(double[] q, double speed = 0, double acceleration = 0, double time = 0.002, 
                          double lookaheadTime = 0.1, double gain = 300)
        {
            ThrowIfDisposed();
            ValidateArray(q, 6, nameof(q));
            CheckStatus(NativeMethods.ur_rtde_control_servo_j(_handle, q, speed, acceleration, time, lookaheadTime, gain));
        }

        /// <summary>
        /// Set tool center point
        /// </summary>
        /// <param name="tcpPose">TCP offset [x, y, z, rx, ry, rz]</param>
        public void SetTcp(double[] tcpPose)
        {
            ThrowIfDisposed();
            ValidateArray(tcpPose, 6, nameof(tcpPose));
            CheckStatus(NativeMethods.ur_rtde_control_set_tcp(_handle, tcpPose));
        }

        /// <summary>
        /// Set payload mass and center of gravity
        /// </summary>
        /// <param name="mass">Payload mass [kg]</param>
        /// <param name="centerOfGravity">Center of gravity [x, y, z] in meters</param>
        public void SetPayload(double mass, double[] centerOfGravity)
        {
            ThrowIfDisposed();
            ValidateArray(centerOfGravity, 3, nameof(centerOfGravity));
            CheckStatus(NativeMethods.ur_rtde_control_set_payload(_handle, mass, centerOfGravity));
        }

        /// <summary>
        /// Reset watchdog timer
        /// </summary>
        public void TriggerWatchdog()
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_trigger_watchdog(_handle));
        }

        private void CheckStatus(NativeMethods.Status status)
        {
            if (status == NativeMethods.Status.OK)
                return;

            var errorPtr = NativeMethods.ur_rtde_control_get_last_error(_handle);
            var errorMsg = NativeMethods.GetString(errorPtr) ?? "Unknown error";

            throw status switch
            {
                NativeMethods.Status.ErrorConnection => new RTDEConnectionException($"Connection error: {errorMsg}"),
                NativeMethods.Status.ErrorTimeout => new RTDEException($"Timeout: {errorMsg}"),
                NativeMethods.Status.ErrorNotConnected => new RTDEConnectionException($"Not connected: {errorMsg}"),
                NativeMethods.Status.ErrorCommandFailed => new RTDEException($"Command failed: {errorMsg}"),
                _ => new RTDEException($"Error {status}: {errorMsg}")
            };
        }

        private static void ValidateArray(double[] array, int expectedLength, string paramName)
        {
            if (array == null)
                throw new ArgumentNullException(paramName);
            if (array.Length != expectedLength)
                throw new ArgumentException($"Expected array of length {expectedLength}, got {array.Length}", paramName);
        }

        private void ThrowIfDisposed()
        {
            if (_disposed)
                throw new ObjectDisposedException(nameof(RTDEControl));
        }

        public void Dispose()
        {
            if (_disposed)
                return;

            if (_handle != IntPtr.Zero)
            {
                NativeMethods.ur_rtde_control_destroy(_handle);
                _handle = IntPtr.Zero;
            }

            _disposed = true;
        }

        ~RTDEControl()
        {
            Dispose();
        }
    }
}
