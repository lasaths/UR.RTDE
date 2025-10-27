using System;
using Python.Runtime;

namespace UR.RTDE.PythonBridge
{
    /// <summary>
    /// RTDE Control Interface using Python.NET bridge to ur_rtde
    /// This implementation uses the Python ur_rtde library via Python.NET
    /// </summary>
    public sealed class RTDEControlPython : IDisposable
    {
        private dynamic? _control;
        private bool _disposed;
        private readonly string _hostname;

        /// <summary>
        /// Create RTDE Control Interface
        /// </summary>
        public RTDEControlPython(string hostname, double frequency = -1.0, ushort flags = 1, int urCapPort = 50002)
        {
            if (string.IsNullOrWhiteSpace(hostname))
                throw new ArgumentNullException(nameof(hostname));

            if (!PythonEngineManager.IsInitialized)
                PythonEngineManager.Initialize();

            _hostname = hostname;

            using (Py.GIL())
            {
                dynamic rtde_control = Py.Import("rtde_control");
                _control = rtde_control.RTDEControlInterface(hostname, frequency, (int)flags, urCapPort);

                if (!_control.isConnected())
                    throw new RTDEConnectionException($"Failed to connect to robot at {hostname}");
            }
        }

        /// <summary>
        /// Check if connected to robot
        /// </summary>
        public bool IsConnected
        {
            get
            {
                ThrowIfDisposed();
                using (Py.GIL())
                {
                    return _control.isConnected();
                }
            }
        }

        /// <summary>
        /// Disconnect from robot
        /// </summary>
        public void Disconnect()
        {
            ThrowIfDisposed();
            using (Py.GIL())
            {
                _control.disconnect();
            }
        }

        /// <summary>
        /// Reconnect to robot
        /// </summary>
        public void Reconnect()
        {
            ThrowIfDisposed();
            using (Py.GIL())
            {
                bool success = _control.reconnect();
                if (!success)
                    throw new RTDEConnectionException($"Failed to reconnect to {_hostname}");
            }
        }

        /// <summary>
        /// Move to joint position (linear in joint-space)
        /// </summary>
        public void MoveJ(double[] q, double speed = 1.05, double acceleration = 1.4, bool asynchronous = false)
        {
            ThrowIfDisposed();
            ValidateArray(q, 6, nameof(q));

            using (Py.GIL())
            {
                using (var pyQ = q.ToPython())
                {
                    bool success = _control.moveJ(pyQ, speed, acceleration, asynchronous);
                    if (!success)
                        throw new RTDEException("MoveJ command failed");
                }
            }
        }

        /// <summary>
        /// Move to pose (linear in tool-space)
        /// </summary>
        public void MoveL(double[] pose, double speed = 0.25, double acceleration = 1.2, bool asynchronous = false)
        {
            ThrowIfDisposed();
            ValidateArray(pose, 6, nameof(pose));

            using (Py.GIL())
            {
                using (var pyPose = pose.ToPython())
                {
                    bool success = _control.moveL(pyPose, speed, acceleration, asynchronous);
                    if (!success)
                        throw new RTDEException("MoveL command failed");
                }
            }
        }

        /// <summary>
        /// Stop robot (linear in joint-space)
        /// </summary>
        public void StopJ(double acceleration = 2.0, bool asynchronous = false)
        {
            ThrowIfDisposed();
            using (Py.GIL())
            {
                _control.stopJ(acceleration, asynchronous);
            }
        }

        /// <summary>
        /// Stop robot (linear in tool-space)
        /// </summary>
        public void StopL(double acceleration = 10.0, bool asynchronous = false)
        {
            ThrowIfDisposed();
            using (Py.GIL())
            {
                _control.stopL(acceleration, asynchronous);
            }
        }

        /// <summary>
        /// Joint speed control
        /// </summary>
        public void SpeedJ(double[] qd, double acceleration = 0.5, double time = 0.0)
        {
            ThrowIfDisposed();
            ValidateArray(qd, 6, nameof(qd));

            using (Py.GIL())
            {
                using (var pyQd = qd.ToPython())
                {
                    bool success = _control.speedJ(pyQd, acceleration, time);
                    if (!success)
                        throw new RTDEException("SpeedJ command failed");
                }
            }
        }

        /// <summary>
        /// Tool speed control
        /// </summary>
        public void SpeedL(double[] xd, double acceleration = 0.25, double time = 0.0)
        {
            ThrowIfDisposed();
            ValidateArray(xd, 6, nameof(xd));

            using (Py.GIL())
            {
                using (var pyXd = xd.ToPython())
                {
                    bool success = _control.speedL(pyXd, acceleration, time);
                    if (!success)
                        throw new RTDEException("SpeedL command failed");
                }
            }
        }

        /// <summary>
        /// Servo to joint position
        /// </summary>
        public void ServoJ(double[] q, double speed = 0, double acceleration = 0, double time = 0.002,
                          double lookaheadTime = 0.1, double gain = 300)
        {
            ThrowIfDisposed();
            ValidateArray(q, 6, nameof(q));

            using (Py.GIL())
            {
                using (var pyQ = q.ToPython())
                {
                    bool success = _control.servoJ(pyQ, speed, acceleration, time, lookaheadTime, gain);
                    if (!success)
                        throw new RTDEException("ServoJ command failed");
                }
            }
        }

        /// <summary>
        /// Set tool center point
        /// </summary>
        public void SetTcp(double[] tcpPose)
        {
            ThrowIfDisposed();
            ValidateArray(tcpPose, 6, nameof(tcpPose));

            using (Py.GIL())
            {
                using (var pyTcp = tcpPose.ToPython())
                {
                    bool success = _control.setTcp(pyTcp);
                    if (!success)
                        throw new RTDEException("SetTcp command failed");
                }
            }
        }

        /// <summary>
        /// Set payload mass and center of gravity
        /// </summary>
        public void SetPayload(double mass, double[] centerOfGravity)
        {
            ThrowIfDisposed();
            ValidateArray(centerOfGravity, 3, nameof(centerOfGravity));

            using (Py.GIL())
            {
                using (var pyCog = centerOfGravity.ToPython())
                {
                    bool success = _control.setPayload(mass, pyCog);
                    if (!success)
                        throw new RTDEException("SetPayload command failed");
                }
            }
        }

        /// <summary>
        /// Reset watchdog timer
        /// </summary>
        public void TriggerWatchdog()
        {
            ThrowIfDisposed();
            using (Py.GIL())
            {
                _control.triggerWatchdog();
            }
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
                throw new ObjectDisposedException(nameof(RTDEControlPython));
        }

        public void Dispose()
        {
            if (_disposed)
                return;

            if (_control != null)
            {
                using (Py.GIL())
                {
                    try
                    {
                        _control.disconnect();
                    }
                    catch { }
                    _control = null;
                }
            }

            _disposed = true;
        }

        ~RTDEControlPython()
        {
            Dispose();
        }
    }
}
