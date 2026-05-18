using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;

namespace UR.RTDE
{
    /// <summary>
    /// Exception thrown when RTDE operations fail
    /// </summary>
    public class RTDEException : Exception
    {
        public RTDEException(string message) : base(message) { }
        public RTDEException(string message, Exception inner) : base(message, inner) { }
    }

    /// <summary>
    /// Exception thrown when connection to robot fails
    /// </summary>
    public class RTDEConnectionException : RTDEException
    {
        public RTDEConnectionException(string message) : base(message) { }
        public RTDEConnectionException(string message, Exception inner) : base(message, inner) { }
    }

    internal static class NativeLoadDiagnostics
    {
        internal static string BuildMessage(string operation, Exception ex)
        {
            string rid = GetRuntimeIdentifier();
            string arch = RuntimeInformation.ProcessArchitecture.ToString();
            string os = RuntimeInformation.OSDescription.Trim();
            string baseDir = AppContext.BaseDirectory;

            return
                $"{operation} failed because the native UR.RTDE library could not be loaded correctly. " +
                $"Runtime: {os}, Arch: {arch}, RID: {rid}. BaseDir: {baseDir}. " +
                $"Make sure the package includes matching native assets under runtimes/{rid}/native " +
                $"(for example libur_rtde_c_api.dylib on macOS or ur_rtde_c_api.dll on Windows). " +
                $"Native error: {ex.Message}";
        }

        internal static string BuildBootstrapMessage(string operation, Exception ex)
        {
            string rid = GetRuntimeIdentifier();
            return
                $"{operation} failed native bootstrap preflight before creating the RTDE client. " +
                $"Expected library: {GetPrimaryNativeLibraryFileName()}, RID: {rid}. " +
                $"Native preflight error: {ex.Message}";
        }

        internal static string GetRuntimeIdentifier()
        {
            string? rid = AppContext.GetData("RUNTIME_IDENTIFIER")?.ToString();
            if (!string.IsNullOrWhiteSpace(rid))
                return rid!;

            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
                return "win-x64";

            if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
                return RuntimeInformation.ProcessArchitecture == Architecture.Arm64 ? "osx-arm64" : "osx-x64";

            if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
                return RuntimeInformation.ProcessArchitecture == Architecture.Arm64 ? "linux-arm64" : "linux-x64";

            return "unknown";
        }

        internal static string GetPrimaryNativeLibraryFileName()
        {
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
                return "ur_rtde_c_api.dll";

            if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
                return "libur_rtde_c_api.dylib";

            return "libur_rtde_c_api.so";
        }
    }

    internal static class NativeBootstrapGuard
    {
        private static readonly object Gate = new object();
        private static bool _complete;

        internal static void EnsureInitialized(string operation)
        {
            lock (Gate)
            {
                if (_complete)
                    return;

                try
                {
                    VerifyPrimaryNativeLibrary();
                }
                catch (Exception ex)
                {
                    throw new RTDEConnectionException(
                        NativeLoadDiagnostics.BuildBootstrapMessage(operation, ex), ex);
                }

                _complete = true;
            }
        }

        private static void VerifyPrimaryNativeLibrary()
        {
            if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
            {
                Native.MacOsNativeLibraryBootstrap.EnsureInitialized();
                return;
            }

            string rid = NativeLoadDiagnostics.GetRuntimeIdentifier();
            string libraryName = NativeLoadDiagnostics.GetPrimaryNativeLibraryFileName();
            List<string> searchedPaths = new();

            foreach (string directory in GetCandidateDirectories(rid))
            {
                string candidate = Path.Combine(directory, libraryName);
                searchedPaths.Add(candidate);
                if (!File.Exists(candidate))
                    continue;

#if NET8_0_OR_GREATER
                if (!NativeLibrary.TryLoad(candidate, out nint handle))
                    throw new DllNotFoundException($"Found native library at '{candidate}' but failed to load it.");

                NativeLibrary.Free(handle);
#endif
                return;
            }

            throw new FileNotFoundException(
                $"Could not locate '{libraryName}'. Searched: {string.Join(", ", searchedPaths)}");
        }

        private static IEnumerable<string> GetCandidateDirectories(string rid)
        {
            string baseDir = AppContext.BaseDirectory;
            string assemblyDir = Path.GetDirectoryName(typeof(NativeBootstrapGuard).Assembly.Location) ?? baseDir;

            yield return baseDir;
            if (!string.Equals(assemblyDir, baseDir, StringComparison.OrdinalIgnoreCase))
                yield return assemblyDir;

            if (!string.IsNullOrWhiteSpace(rid) && !string.Equals(rid, "unknown", StringComparison.OrdinalIgnoreCase))
            {
                yield return Path.Combine(baseDir, "runtimes", rid, "native");
                if (!string.Equals(assemblyDir, baseDir, StringComparison.OrdinalIgnoreCase))
                    yield return Path.Combine(assemblyDir, "runtimes", rid, "native");
            }
        }
    }
}
