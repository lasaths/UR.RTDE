# Changelog

All notable changes to UR.RTDE will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Planned
- macOS arm64 native library support
- Grasshopper components for Rhino 7/8
- GitHub Actions CI/CD pipeline
- Dashboard client wrapper
- Script client wrapper
- Publish to NuGet.org

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
