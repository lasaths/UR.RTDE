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
            string primaryRid = RuntimeInformation.ProcessArchitecture == Architecture.Arm64 ? "osx-arm64" : "osx-x64";
            string secondaryRid = primaryRid == "osx-arm64" ? "osx-x64" : "osx-arm64";
            string runtimes = Path.Combine(assemblyDir, "runtimes");

            // Prefer RID-specific runtimes first so a wrong-arch flat copy (e.g. arm64 beside the GHA on Rosetta) is not chosen.
            string[] candidates =
            {
                Path.Combine(runtimes, primaryRid, "native", fileName),
                Path.Combine(runtimes, secondaryRid, "native", fileName),
                Path.Combine(assemblyDir, fileName),
            };

            foreach (string candidate in candidates)
            {
                if (!File.Exists(candidate))
                    continue;

                if (candidate.StartsWith(runtimes, StringComparison.Ordinal) || IsMachOCompatibleWithProcess(candidate))
                    return candidate;
            }

            return null;
        }

        private static bool IsMachOCompatibleWithProcess(string path)
        {
            try
            {
                using var stream = File.OpenRead(path);
                var header = new byte[8];
                if (stream.Read(header, 0, header.Length) < 8)
                    return false;

                // MH_MAGIC_64 (little-endian): 0xFE 0xED 0xFA 0xCF; cputype at offset 4.
                if (header[0] != 0xCF || header[1] != 0xFA || header[2] != 0xED || header[3] != 0xFE)
                    return true;

                uint cpuType = BitConverter.ToUInt32(header, 4);
                const uint CPU_TYPE_X86_64 = 0x01000007;
                const uint CPU_TYPE_ARM64 = 0x0100000c;

                return RuntimeInformation.ProcessArchitecture switch
                {
                    Architecture.X64 => cpuType == CPU_TYPE_X86_64,
                    Architecture.Arm64 => cpuType == CPU_TYPE_ARM64,
                    _ => true,
                };
            }
            catch
            {
                return true;
            }
        }
    }
}
