using System;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Threading;

namespace UR.RTDE.Native
{
    /// <summary>
    /// Loads the UR.RTDE C facade on macOS and wires P/Invoke to that handle.
    /// The facade contains ur_rtde and Boost internally and exports only the C ABI.
    /// </summary>
    public static class MacOsNativeLibraryBootstrap
    {
        private const int RTLD_LAZY = 0x1;
        private const int RTLD_LOCAL = 0x4;
        // Prefer resolving symbols in this image first (Darwin).
        private const int RTLD_FIRST = 0x100;
        private const string PrimaryNativeLibrary = "libur_rtde_c_api.dylib";

        private static readonly Lazy<bool> NativeInit = new(InitializeNative, LazyThreadSafetyMode.ExecutionAndPublication);

        private static IntPtr _cApiHandle;
        private static string? _lastError;

        /// <summary>Last dlopen error message, if any.</summary>
        public static string? LastLoadError => _lastError;

        /// <summary>
        /// Preloads the native dependency chain and registers a DllImport resolver on .NET 8+.
        /// Safe to call multiple times.
        /// </summary>
        public static void EnsureInitialized()
        {
            if (!RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
                return;

            // Must complete before any P/Invoke: do not use CompareExchange-before-work — other threads can
            // otherwise return early while dlopen / SetDllImportResolver is still running (Connect uses TP workers).
            _ = NativeInit.Value;
        }

        private static bool InitializeNative()
        {
            try
            {
                PreloadDependencies();
#if NET8_0_OR_GREATER
                NativeLibrary.SetDllImportResolver(typeof(NativeMethods).Assembly, ResolveNativeLibrary);
#endif
            }
            catch
            {
                _cApiHandle = IntPtr.Zero;
                throw;
            }

            return true;
        }

        private static void PreloadDependencies()
        {
            string? assemblyDir = Path.GetDirectoryName(typeof(MacOsNativeLibraryBootstrap).Assembly.Location);
            if (string.IsNullOrWhiteSpace(assemblyDir))
                throw new DllNotFoundException("Could not determine UR.RTDE assembly directory for native libraries.");

            int flags = RTLD_LAZY | RTLD_LOCAL | RTLD_FIRST;
            string? path = ResolveLibraryPath(assemblyDir, PrimaryNativeLibrary);
            if (string.IsNullOrWhiteSpace(path))
            {
                _lastError = $"Native dependency not found: {PrimaryNativeLibrary}";
                throw new DllNotFoundException(
                    $"Failed to load macOS native dependency '{PrimaryNativeLibrary}'. {_lastError} | assemblyDir={assemblyDir}");
            }

            _ = dlerror();
            _cApiHandle = dlopen(path!, flags);
            if (_cApiHandle == IntPtr.Zero)
            {
                _lastError = Marshal.PtrToStringAnsi(dlerror()) ?? "unknown dlopen error";
                throw new DllNotFoundException(
                    $"Failed to load macOS native dependency '{PrimaryNativeLibrary}'. {_lastError} | path={path}");
            }
        }

#if NET8_0_OR_GREATER
        private static IntPtr ResolveNativeLibrary(
            string libraryName,
            Assembly assembly,
            DllImportSearchPath? searchPath)
        {
            if (IsPrimaryNativeLibraryName(libraryName))
            {
                if (_cApiHandle == IntPtr.Zero)
                    throw new DllNotFoundException(
                        "UR.RTDE macOS native library is not initialized. Call MacOsNativeLibraryBootstrap.EnsureInitialized() first.");
                return _cApiHandle;
            }

            return IntPtr.Zero;
        }
#endif

        internal static bool IsPrimaryNativeLibraryName(string libraryName)
        {
            return string.Equals(libraryName, "ur_rtde_c_api", StringComparison.OrdinalIgnoreCase)
                || string.Equals(libraryName, PrimaryNativeLibrary, StringComparison.OrdinalIgnoreCase);
        }

        [DllImport("libSystem.dylib", EntryPoint = "dlopen")]
        private static extern IntPtr dlopen(string path, int mode);

        [DllImport("libSystem.dylib", EntryPoint = "dlerror")]
        private static extern IntPtr dlerror();

        private static string? ResolveLibraryPath(string assemblyDir, string fileName)
        {
            string rid = RuntimeInformation.ProcessArchitecture == Architecture.Arm64 ? "osx-arm64" : "osx-x64";
            string runtimes = Path.Combine(assemblyDir, "runtimes");

            string[] candidates =
            {
                Path.Combine(assemblyDir, fileName),
                Path.Combine(runtimes, rid, "native", fileName),
                Path.Combine(runtimes, "osx-arm64", "native", fileName),
                Path.Combine(runtimes, "osx-x64", "native", fileName)
            };

            foreach (string candidate in candidates)
            {
                if (File.Exists(candidate))
                    return candidate;
            }

            return null;
        }
    }
}
