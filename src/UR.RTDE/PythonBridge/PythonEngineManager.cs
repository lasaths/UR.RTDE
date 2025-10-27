using System;
using Python.Runtime;

namespace UR.RTDE.PythonBridge
{
    /// <summary>
    /// Manages Python runtime initialization for ur_rtde bridge
    /// </summary>
    public static class PythonEngineManager
    {
        private static bool _initialized = false;
        private static readonly object _lock = new object();

        /// <summary>
        /// Initialize Python runtime (call once at startup)
        /// </summary>
        public static void Initialize()
        {
            lock (_lock)
            {
                if (_initialized)
                    return;

                // Set Python DLL path for Windows
                if (Environment.OSVersion.Platform == PlatformID.Win32NT)
                {
                    Runtime.PythonDLL = @"C:\Users\lasaths\miniconda3\python312.dll";
                }

                Python.Runtime.PythonEngine.Initialize();
                _initialized = true;
            }
        }

        /// <summary>
        /// Shutdown Python runtime (call on application exit)
        /// </summary>
        public static void Shutdown()
        {
            lock (_lock)
            {
                if (!_initialized)
                    return;

                Python.Runtime.PythonEngine.Shutdown();
                _initialized = false;
            }
        }

        /// <summary>
        /// Check if Python runtime is initialized
        /// </summary>
        public static bool IsInitialized => _initialized;
    }
}
