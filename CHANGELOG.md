# Changelog

All notable changes to UR.RTDE will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
Package versions **track upstream [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde)** (`MAJOR.MINOR.PATCH`, optional fourth segment for wrapper-only fixes).

---

## [Unreleased]

### Planned
- Grasshopper components for Rhino 7/8
- GitHub Actions CI/CD pipeline
- Dashboard client wrapper
- Script client wrapper
- Publish to NuGet.org

---

## [1.6.3.10] - 2026-05-19

### Added
- **osx-x64** native runtime (`libur_rtde_c_api.dylib`) for macOS Intel and Rosetta (x86_64) Rhino 8 / .NET 8 hosts.
- `native/build-macos-x64-static-boost.sh` — static Boost + ur_rtde facade build for x86_64.

### Fixed
- `MacOsNativeLibraryBootstrap` resolves `runtimes/osx-arm64` or `runtimes/osx-x64` before a flat dylib beside the assembly, and skips a flat Mach-O image when CPU type does not match the process (avoids arm64-on-Rosetta load failures).

---

## [1.6.3.9] - 2026-05-18

### Fixed
- macOS Apple Silicon / Rhino 8: validated end-to-end native load and RTDE connect with unified `libur_rtde_c_api.dylib` and thread-safe bootstrap (builds on 1.6.3.8).

### Added
- Restored **win-x64** native runtimes in the NuGet package (`ur_rtde_c_api.dll`, `rtde.dll`, Boost thread DLLs) so Windows Rhino 7/8 deployments again receive RID-specific binaries alongside **osx-arm64**.

### Changed
- macOS arm64 package ships only `libur_rtde_c_api.dylib` (no separate `librtde` or Boost dylibs).

---

## [1.6.3.8] - 2026-05-18

### Fixed
- macOS native bootstrap race: `Connect` can run on a thread-pool worker in Grasshopper, so initialization now uses a thread-safe one-shot path before any P/Invoke call can enter `ur_rtde_receive_create` or related native constructors.
- Native preflight now waits for the macOS `dlopen` sequence and .NET 8 `DllImport` resolver registration to complete before marking bootstrap complete.

### Changed
- macOS arm64 runtime now ships as a single `libur_rtde_c_api.dylib` with `ur_rtde` and Boost linked internally and C++ symbols hidden; no separate `librtde*.dylib` or Boost dylibs should be deployed for Rhino.
- macOS native loading uses `RTLD_LAZY | RTLD_LOCAL | RTLD_FIRST` for the facade to reduce accidental symbol binding in Rhino 8 processes.
- macOS runtime package removes separate `librtde` and Boost dylibs from the NuGet runtime payload.

### Notes
- Rhino installed on a non-standard volume path is supported; the relevant deploy location is the Grasshopper Libraries folder that contains `UR.RTDE.dll` and the `runtimes/osx-arm64/native/` tree.
- End-to-end Rhino 8 Connect validation still requires a full Rhino restart after replacing the plugin and native dylibs.

---

## [1.6.3.3] - 2026-05-16

### Added
- Native bootstrap preflight guard before constructing RTDE native clients (`RTDEControl`, `RTDEReceive`, `RTDEIO`, `RobotiqGripperNative`) with clearer load-failure diagnostics.

### Changed
- Native source workflow now pins `ur_rtde` to commit `68ac4e18f357f8e9361bfc5eef344acd9aa241be` for reproducible builds.
- Rebuilt macOS arm64 runtime dylibs from pinned source and normalized linkage to `@loader_path` for deployable in-package loading.

### Notes
- This release supersedes `1.6.3.2` because that package version was already published.

---

## [1.6.3.0] - 2026-05-16

### Changed
- **Versioning policy:** NuGet package version now matches bundled **ur_rtde 1.6.3** (replaces independent `1.3.0` on the feed).

### Added
- macOS Apple Silicon (`osx-arm64`) native runtimes: `libur_rtde_c_api.dylib`, `librtde` 1.6.3, and Boost 1.85 (`system`/`thread`) with `@loader_path` linkage.

### Fixed
- C API header: `#include <stddef.h>` for `size_t` when building the facade with Apple Clang.
- README: release instructions, test paths, build doc links, and `master` branch name.

### Notes
- Use Boost **1.85** on macOS when rebuilding natives (Boost 1.89 removes `boost_system` CMake target).

---

## [1.3.0.0] - 2026-05-16 (superseded by 1.6.3.0)

Same payload as 1.6.3.0; use **1.6.3.0** for version alignment with ur_rtde.

---

## [1.2.0.0] - 2025-12-11

### Changed
- Upstream ur_rtde bumped to v1.6.2; native libraries rebuilt with VS 2026 toolset.
- Facade/API: added IO-based `set_input_{int,double}_register` and removed obsolete control-side register setters and output bit register entrypoint.
- Robotiq RTDE path now uses IO register setters; constructor updated to require `RTDEIO`.
- Runtimes now include `boost_thread-vc145-mt-x64-1_89.dll` alongside `rtde.dll` and `ur_rtde_c_api.dll`.

### Fixed/Notes
- URSim integration suite passing (22 core + 6 advanced) against URSim e-Series 5.23.0 @ localhost.

---

## [1.1.1.0] - 2025-09-12

### Changed
- CI simplified to Windows-only with corrected vcpkg/toolchain wiring.
- Clarified native P/Invoke intent in managed wrappers and standardized RobotiqNative dispose pattern.
- Documentation refreshed for the latest release workflow.

### Fixed/Notes
- Resolved vcpkg invocation path issues in CI.
- URSim integration tests validated locally (22 core + 6 advanced).

---

## [1.1.0.0] - 2025-10-28

### Added
- Robotiq Gripper Support
  - Option 2 (URScript client): `URScriptClient` + `RobotiqGripper` with `rq_activate`, `rq_open`, `rq_close`, `rq_move`, `rq_set_speed`, `rq_set_force`.
  - Option 3 (Fast RTDE-register bridge): `RobotiqGripperRtde` with one-time URScript bridge and low-latency control via RTDE input/output registers (SDU-style indices).
- Native C API expansions (facade):
  - RTDEControl: `set_input_int/double/bit_register`, `send_custom_script`.
  - RTDEReceive: `get_output_int/double/bit_register`.
- C# wrappers for register access and custom script upload.
- Test gating via environment variables: `ENABLE_ROBOTIQ_TESTS`, `ENABLE_FT_TESTS`.
- Samples updated to include Robotiq (gated) and register access examples.

### Changed
- Bumped package version to 1.1.0.0 and updated release notes.
- README/Docs updated with Robotiq usage (URCap requirement) and test flags.
- Minor hardening of integration tests and environment configurability.

### Fixed/Notes
- FT zeroing in URSim can be unavailable; test gated via `ENABLE_FT_TESTS`.
- Intermittent first-connect hiccup observed in some environments; subsequent connects succeed.

### Technical Details
- Built on ur_rtde v1.6.0 by SDU Robotics.
- Native facade adds register and script endpoints; managed layer exposes safe wrappers.
- URSim e-Series 5.23.0 validated; core suite passes, Robotiq tests require URCap.

---

## [1.0.0] - 2025-10-27

### Added
- Initial release with basic functionality
- **RTDEControl** basic features:
  - Movement: `MoveJ`, `MoveL`, `SpeedJ`, `SpeedL`, `ServoJ`
  - Stop: `StopJ`, `StopL`
  - Configuration: `SetTcp`, `SetPayload`
  - Connection: `Connect`, `Disconnect`, `IsConnected`, `Reconnect`
- **RTDEReceive** basic features:
  - Position: `GetActualQ`, `GetActualTcpPose`
  - Status: `GetRobotMode`, `GetSafetyMode`
  - Connection: `Connect`, `Disconnect`, `IsConnected`, `Reconnect`
- **Native build infrastructure**:
  - CMake-based build system
  - vcpkg integration for Boost
  - C API facade (ur_rtde_c_api.dll)
  - Multi-TFM support (net48, net8.0)
- **NuGet packaging**:
  - Automatic DLL deployment via `runtimes/{rid}/native/` structure
  - No manual DLL copying required
- **Documentation**:
  - [README.md](README.md) - Quick start and installation
  - [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) - Manual build guide
  - [AGENTS.md](AGENTS.md) - Agent/developer instructions
### Technical Details
- Built on ur_rtde v1.6.0 by SDU Robotics
- Target platforms: Windows x64 (macOS arm64 planned)
- .NET targets: .NET Framework 4.8, .NET 8.0
- Architecture: Native C++ P/Invoke (zero Python dependency)

---

## Version History

| Version | Date | ur_rtde | Status | Key Features |
|---------|------|---------|--------|--------------|
| **1.1.1.0** | 2025-09-12 | v1.6.0 | [OK] Maintenance | CI fixes, dispose pattern, docs refresh |
| **1.1.0.0** | 2025-10-28 | v1.6.0 | [OK] Feature Update | Robotiq support, register APIs, docs |
| **1.0.0** | 2025-10-27 | v1.6.0 | Initial Release | Basic control/receive, NuGet packaging |

---

## Upgrade Guide

### From 1.0.0 to 1.1.0.0

**Breaking Changes**: None (fully backward compatible)

**New Features Available**:
- Extended movement control (MoveC, ServoC, SpeedStop, ServoStop)
- Kinematics (IK/FK, safety limits checking)
- Status properties (IsProgramRunning, IsSteady)
- Extended receive data (velocities, accelerations, forces, temperatures, currents)
- Safety monitoring (IsProtectiveStopped, IsEmergencyStopped)
- Digital I/O reading
- New RTDEIO class for I/O control and analog output

**Migration Steps**:
1. Update NuGet package: `dotnet add package UR.RTDE --version 1.1.0.0`
2. No code changes required (existing code continues to work)
3. Optionally use new features:

```csharp
// New kinematics features
using var control = new RTDEControl("192.168.1.100");
double[] pose = { 0.3, 0.0, 0.4, 0, 3.14, 0 };
double[] joints = control.GetInverseKinematics(pose);

// New safety monitoring
using var receive = new RTDEReceive("192.168.1.100");
if (receive.IsProtectiveStopped)
    Console.WriteLine("Robot is in protective stop!");

// New I/O control
using var io = new RTDEIO("192.168.1.100");
io.SetStandardDigitalOut(0, true);  // Set digital output 0
```

---

## Links

- **Repository**: https://github.com/lasaths/UR.RTDE
- **Issues**: https://github.com/lasaths/UR.RTDE/issues
- **NuGet** (planned): https://www.nuget.org/packages/UR.RTDE
- **ur_rtde upstream**: https://gitlab.com/sdurobotics/ur_rtde
- **License**: MIT
