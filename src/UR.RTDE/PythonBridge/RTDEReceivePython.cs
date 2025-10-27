using System;
using System.Linq;
using Python.Runtime;

namespace UR.RTDE.PythonBridge
{
    /// <summary>
    /// RTDE Receive Interface using Python.NET bridge to ur_rtde
    /// </summary>
    public sealed class RTDEReceivePython : IDisposable
    {
        private dynamic? _receive;
        private bool _disposed;

        /// <summary>
        /// Create RTDE Receive Interface
        /// </summary>
        public RTDEReceivePython(string hostname, double frequency = -1.0)
        {
            if (string.IsNullOrWhiteSpace(hostname))
                throw new ArgumentNullException(nameof(hostname));

            if (!PythonEngineManager.IsInitialized)
                PythonEngineManager.Initialize();

            using (Py.GIL())
            {
                dynamic rtde_receive = Py.Import("rtde_receive");
                _receive = rtde_receive.RTDEReceiveInterface(hostname, frequency);

                if (!_receive.isConnected())
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
                    return _receive.isConnected();
                }
            }
        }

        /// <summary>
        /// Get actual joint positions
        /// </summary>
        public double[] GetActualQ()
        {
            ThrowIfDisposed();
            using (Py.GIL())
            {
                dynamic pyResult = _receive.getActualQ();
                return ConvertPythonList(pyResult);
            }
        }

        /// <summary>
        /// Get actual TCP pose
        /// </summary>
        public double[] GetActualTcpPose()
        {
            ThrowIfDisposed();
            using (Py.GIL())
            {
                dynamic pyResult = _receive.getActualTCPPose();
                return ConvertPythonList(pyResult);
            }
        }

        /// <summary>
        /// Get actual joint speeds
        /// </summary>
        public double[] GetActualQd()
        {
            ThrowIfDisposed();
            using (Py.GIL())
            {
                dynamic pyResult = _receive.getActualQd();
                return ConvertPythonList(pyResult);
            }
        }

        /// <summary>
        /// Get actual TCP speed
        /// </summary>
        public double[] GetActualTcpSpeed()
        {
            ThrowIfDisposed();
            using (Py.GIL())
            {
                dynamic pyResult = _receive.getActualTCPSpeed();
                return ConvertPythonList(pyResult);
            }
        }

        /// <summary>
        /// Get robot mode
        /// </summary>
        public int GetRobotMode()
        {
            ThrowIfDisposed();
            using (Py.GIL())
            {
                return (int)_receive.getRobotMode();
            }
        }

        /// <summary>
        /// Get safety mode
        /// </summary>
        public int GetSafetyMode()
        {
            ThrowIfDisposed();
            using (Py.GIL())
            {
                return (int)_receive.getSafetyMode();
            }
        }

        /// <summary>
        /// Get runtime state
        /// </summary>
        public int GetRuntimeState()
        {
            ThrowIfDisposed();
            using (Py.GIL())
            {
                return (int)_receive.getRuntimeState();
            }
        }

        /// <summary>
        /// Get standard digital input state
        /// </summary>
        public bool GetStandardDigitalIn(int index)
        {
            ThrowIfDisposed();
            if (index < 0 || index > 7)
                throw new ArgumentOutOfRangeException(nameof(index), "Index must be 0-7");

            using (Py.GIL())
            {
                return (bool)_receive.getStandardDigitalIn(index);
            }
        }

        /// <summary>
        /// Get standard digital output state
        /// </summary>
        public bool GetStandardDigitalOut(int index)
        {
            ThrowIfDisposed();
            if (index < 0 || index > 7)
                throw new ArgumentOutOfRangeException(nameof(index), "Index must be 0-7");

            using (Py.GIL())
            {
                return (bool)_receive.getStandardDigitalOut(index);
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
                _receive.disconnect();
            }
        }

        private static double[] ConvertPythonList(dynamic pyList)
        {
            // Convert Python list to C# array
            var list = ((PyObject)pyList).As<double[]>();
            return list;
        }

        private void ThrowIfDisposed()
        {
            if (_disposed)
                throw new ObjectDisposedException(nameof(RTDEReceivePython));
        }

        public void Dispose()
        {
            if (_disposed)
                return;

            if (_receive != null)
            {
                using (Py.GIL())
                {
                    try
                    {
                        _receive.disconnect();
                    }
                    catch { }
                    _receive = null;
                }
            }

            _disposed = true;
        }

        ~RTDEReceivePython()
        {
            Dispose();
        }
    }
}
