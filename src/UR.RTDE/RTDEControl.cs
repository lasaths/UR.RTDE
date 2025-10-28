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
        /// Kick/reset the watchdog timer
        /// </summary>
        public void TriggerWatchdog()
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_kick_watchdog(_handle));
        }

        // ====================================================================
        // Kinematics
        // ====================================================================

        /// <summary>
        /// Calculate inverse kinematics - convert TCP pose to joint positions
        /// </summary>
        /// <param name="pose">TCP pose [x, y, z, rx, ry, rz] in meters and radians</param>
        /// <returns>Joint positions [6] in radians, or throws if no solution</returns>
        public double[] GetInverseKinematics(double[] pose)
        {
            ThrowIfDisposed();
            ValidateArray(pose, 6, nameof(pose));
            var q = new double[6];
            CheckStatus(NativeMethods.ur_rtde_control_get_inverse_kinematics(_handle, pose, (UIntPtr)6, q, (UIntPtr)6));
            return q;
        }

        /// <summary>
        /// Calculate forward kinematics - convert joint positions to TCP pose
        /// </summary>
        /// <param name="q">Joint positions [6] in radians</param>
        /// <returns>TCP pose [x, y, z, rx, ry, rz] in meters and radians</returns>
        public double[] GetForwardKinematics(double[] q)
        {
            ThrowIfDisposed();
            ValidateArray(q, 6, nameof(q));
            var pose = new double[6];
            CheckStatus(NativeMethods.ur_rtde_control_get_forward_kinematics(_handle, q, (UIntPtr)6, pose, (UIntPtr)6));
            return pose;
        }

        /// <summary>
        /// Check if inverse kinematics has a valid solution for given pose
        /// </summary>
        /// <param name="pose">TCP pose [x, y, z, rx, ry, rz] to test</param>
        /// <returns>True if IK solution exists, false otherwise</returns>
        public bool HasInverseKinematicsSolution(double[] pose)
        {
            ThrowIfDisposed();
            ValidateArray(pose, 6, nameof(pose));
            return NativeMethods.ur_rtde_control_get_inverse_kinematics_has_solution(_handle, pose, (UIntPtr)6);
        }

        // ====================================================================
        // Advanced Movement
        // ====================================================================

        /// <summary>
        /// Servo in Cartesian space (circular blending)
        /// </summary>
        /// <param name="pose">Target TCP pose [x, y, z, rx, ry, rz]</param>
        /// <param name="speed">Tool speed [m/s]</param>
        /// <param name="acceleration">Tool acceleration [m/s^2]</param>
        /// <param name="blend">Blend radius [m] for path smoothing</param>
        public void ServoC(double[] pose, double speed = 0.25, double acceleration = 1.2, double blend = 0.0)
        {
            ThrowIfDisposed();
            ValidateArray(pose, 6, nameof(pose));
            CheckStatus(NativeMethods.ur_rtde_control_servo_c(_handle, pose, speed, acceleration, blend));
        }

        /// <summary>
        /// Stop servo movement
        /// </summary>
        /// <param name="acceleration">Deceleration [rad/s^2 or m/s^2]</param>
        public void ServoStop(double acceleration = 10.0)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_servo_stop(_handle, acceleration));
        }

        /// <summary>
        /// Stop speed movement (SpeedJ/SpeedL)
        /// </summary>
        /// <param name="acceleration">Deceleration [rad/s^2 or m/s^2]</param>
        public void SpeedStop(double acceleration = 10.0)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_speed_stop(_handle, acceleration));
        }

        // ====================================================================
        // Status & Safety
        // ====================================================================

        /// <summary>
        /// Check if a program is currently running on the robot
        /// </summary>
        public bool IsProgramRunning
        {
            get
            {
                if (_disposed) return false;
                return NativeMethods.ur_rtde_control_is_program_running(_handle);
            }
        }

        /// <summary>
        /// Check if robot is in steady state (not moving)
        /// </summary>
        public bool IsSteady
        {
            get
            {
                if (_disposed) return false;
                return NativeMethods.ur_rtde_control_is_steady(_handle);
            }
        }

        /// <summary>
        /// Get detailed robot status bits
        /// </summary>
        /// <returns>Robot status bitmask</returns>
        public uint GetRobotStatus()
        {
            ThrowIfDisposed();
            return NativeMethods.ur_rtde_control_get_robot_status(_handle);
        }

        /// <summary>
        /// Enter force mode for compliant control
        /// </summary>
        /// <param name="taskFrame">Task frame [x, y, z, rx, ry, rz]</param>
        /// <param name="selectionVector">Compliant axes [Fx, Fy, Fz, Mx, My, Mz] (0=position control, 1=force control)</param>
        /// <param name="wrench">Target wrench [Fx, Fy, Fz, Mx, My, Mz] in task frame</param>
        /// <param name="type">Force mode type (1=no transform, 2=base frame, 3=tool frame)</param>
        /// <param name="limits">Max deviation [x, y, z, rx, ry, rz] from start pose</param>
        public void ForceMode(double[] taskFrame, int[] selectionVector, double[] wrench, int type, double[] limits)
        {
            ThrowIfDisposed();
            ValidateArray(taskFrame, 6, nameof(taskFrame));
            if (selectionVector == null || selectionVector.Length != 6)
                throw new ArgumentException("Selection vector must have 6 elements", nameof(selectionVector));
            ValidateArray(wrench, 6, nameof(wrench));
            ValidateArray(limits, 6, nameof(limits));
            
            CheckStatus(NativeMethods.ur_rtde_control_force_mode(
                _handle,
                taskFrame, (UIntPtr)taskFrame.Length,
                selectionVector, (UIntPtr)selectionVector.Length,
                wrench, (UIntPtr)wrench.Length,
                type,
                limits, (UIntPtr)limits.Length));
        }

        /// <summary>
        /// Stop force mode
        /// </summary>
        public void ForceModeStop()
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_force_mode_stop(_handle));
        }

        /// <summary>
        /// Zero the force/torque sensor
        /// </summary>
        public void ZeroFtSensor()
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_zero_ft_sensor(_handle));
        }

        /// <summary>
        /// Start jogging the robot
        /// </summary>
        /// <param name="speeds">Joint or TCP speeds [6]</param>
        /// <param name="feature">Feature (0=base, 1=tool, 2=base rotated, custom features 128-255)</param>
        public void JogStart(double[] speeds, int feature = 0)
        {
            ThrowIfDisposed();
            ValidateArray(speeds, 6, nameof(speeds));
            CheckStatus(NativeMethods.ur_rtde_control_jog_start(_handle, speeds, (UIntPtr)speeds.Length, feature));
        }

        /// <summary>
        /// Stop jogging
        /// </summary>
        public void JogStop()
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_jog_stop(_handle));
        }

        /// <summary>
        /// Enter teach mode (freedrive)
        /// </summary>
        public void TeachMode()
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_teach_mode(_handle));
        }

        /// <summary>
        /// Exit teach mode
        /// </summary>
        public void EndTeachMode()
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_end_teach_mode(_handle));
        }

        /// <summary>
        /// Servo to Cartesian pose (tool space)
        /// </summary>
        /// <param name="pose">Target pose [x, y, z, rx, ry, rz]</param>
        /// <param name="speed">NOT used</param>
        /// <param name="acceleration">NOT used</param>
        /// <param name="time">Command duration [s]</param>
        /// <param name="lookaheadTime">Smoothing [0.03-0.2 s]</param>
        /// <param name="gain">Proportional gain [100-2000]</param>
        public void ServoL(double[] pose, double speed = 0, double acceleration = 0, double time = 0.002,
            double lookaheadTime = 0.1, double gain = 300)
        {
            ThrowIfDisposed();
            ValidateArray(pose, 6, nameof(pose));
            CheckStatus(NativeMethods.ur_rtde_control_servo_l(
                _handle, pose, (UIntPtr)pose.Length, speed, acceleration, time, lookaheadTime, gain));
        }

        /// <summary>
        /// Trigger a protective stop (for testing)
        /// </summary>
        public void TriggerProtectiveStop()
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_trigger_protective_stop(_handle));
        }

        /// <summary>
        /// Set force mode damping parameter
        /// </summary>
        public void ForceModeSetDamping(double damping)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_force_mode_set_damping(_handle, damping));
        }

        /// <summary>
        /// Set force mode gain scaling
        /// </summary>
        public void ForceModeSetGainScaling(double scaling)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_force_mode_set_gain_scaling(_handle, scaling));
        }

        // ====================================================================
        // RTDE Registers & Custom Script
        // ====================================================================

        public void SetInputIntRegister(ushort reg, int value)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_set_input_int_register(_handle, reg, value));
        }

        public void SetInputDoubleRegister(ushort reg, double value)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_set_input_double_register(_handle, reg, value));
        }

        public void SetInputBitRegister(ushort reg, bool value)
        {
            ThrowIfDisposed();
            CheckStatus(NativeMethods.ur_rtde_control_set_input_bit_register(_handle, reg, value));
        }

        public void SendCustomScript(string script)
        {
            ThrowIfDisposed();
            if (string.IsNullOrWhiteSpace(script)) throw new ArgumentNullException(nameof(script));
            CheckStatus(NativeMethods.ur_rtde_control_send_custom_script(_handle, script));
        }

        // ====================================================================
        // Helper Methods
        // ====================================================================

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
