# Version Matrix

| Component | Version | Commit/Tag | Date |
|-----------|---------|------------|------|
| **UR.RTDE package** | 1.6.3.9 | macOS Apple Silicon validated; win-x64 runtimes restored | 2026-05-18 |
| **ur_rtde** | 1.6.3 (pinned commit) | [68ac4e1](https://gitlab.com/sdurobotics/ur_rtde/-/commit/68ac4e18f357f8e9361bfc5eef344acd9aa241be) | 2026-05-16 |
| **License** | MIT | - | - |

## Target Platforms

| Platform | Architecture | .NET Framework | Status |
|----------|--------------|----------------|--------|
| Windows | x64 | .NET 4.8 (Rhino 7) | Supported |
| Windows | x64 | .NET 8.0 (Rhino 8) | Supported |
| macOS | arm64 | .NET 8.0 (Rhino 8) | Supported (single `libur_rtde_c_api.dylib`, validated 1.6.3.9) |
| macOS | x64 (Rosetta) | .NET 8.0 (Rhino 8) | Optional |

## Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| Boost | 1.85 for macOS static build; vcpkg Boost on Windows | ur_rtde dependency |
| CMake | >=3.15 | Native build |
| .NET SDK | 8.0 | Managed build |

## macOS Rhino Notes

- Use UR.RTDE 1.6.3.9 or newer for Rhino 8 on Apple Silicon.
- The `osx-arm64` package includes only `libur_rtde_c_api.dylib` (ur_rtde and Boost linked internally); do not deploy separate `librtde` or Boost dylibs.
- Rhino may be installed outside `/Applications`; native loading is based on the Grasshopper Libraries deploy folder and the `runtimes/osx-arm64/native/` subtree.

## UR Robot Compatibility

| Robot Series | PolyScope Version | Status |
|--------------|-------------------|--------|
| CB3/CB3.1 | >=3.3 | [OK] Supported |
| e-Series | All | [OK] Supported |
| UR+ Series | >=5.0 | [OK] Supported |

## RTDE Configuration

| Parameter | Default | Notes |
|-----------|---------|-------|
| Frequency | 500 Hz | e-Series/UR-Series |
| Frequency | 125 Hz | CB-Series |
| Port | 30004 | RTDE protocol |
| Protocol | TCP/IP | - |
