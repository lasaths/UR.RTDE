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

## [2.0.0] - 2025-10-28

### Added
- **Complete API coverage**: 61 methods across 3 interfaces (Control, Receive, I/O)
- **RTDEControl** extended features (13 methods total):
  - Movement: `MoveJ`, `MoveL`, `MoveC`, `SpeedJ`, `SpeedL`, `ServoJ`, `ServoC`
  - Stop modes: `StopJ`, `StopL`, `ServoStop`, `SpeedStop`
  - Configuration: `SetTcp`, `SetPayload`
  - Kinematics: `GetInverseKinematics`, `GetForwardKinematics`, `IsPoseWithinSafetyLimits`
  - Status: `IsProgramRunning` property, `IsSteady` property
- **RTDEReceive** extended features (16 methods total):
  - Position data: `GetActualQ`, `GetTargetQ`, `GetActualQd`, `GetTargetQd`, `GetActualQdd`, `GetTargetQdd`
  - TCP data: `GetActualTcpPose`, `GetTargetTcpPose`, `GetActualTcpSpeed`
  - Force/torque: `GetActualTcpForce`, `GetTargetMoment`
  - Monitoring: `GetJointTemperatures`, `GetActualCurrent`, `GetTargetCurrent`
  - Status: `GetRobotMode`, `GetSafetyMode`, `GetRuntimeState`
  - Safety: `IsProtectiveStopped` property, `IsEmergencyStopped` property
  - Digital I/O: `GetActualDigitalInputBits`, `GetActualDigitalOutputBits`
- **RTDEIO** new class (6 methods):
  - Digital I/O: `SetStandardDigitalOut`, `SetToolDigitalOut`
  - Analog output: `SetStandardAnalogOut`, `SetAnalogOutputVoltage`, `SetAnalogOutputCurrent`
  - Control: `SetSpeedSlider`
- **Comprehensive documentation**:
  - [UPDATING_URRTDE.md](UPDATING_URRTDE.md) - Complete guide for updating to newer ur_rtde versions
  - [TEST_REPORT.md](TEST_REPORT.md) - URSim validation results (7/7 tests passed)
  - [BUILD_SUCCESS.md](BUILD_SUCCESS.md) - Build process documentation
  - [FEATURE_COVERAGE.md](FEATURE_COVERAGE.md) - API coverage matrix
- **URSim validation**: All features tested with URSim e-Series 5.23.0 @ 172.18.0.2

### Changed
- Upgraded to **production-ready status** after comprehensive testing
- Enhanced error handling with detailed exception messages
- Improved P/Invoke marshaling for array parameters
- Updated documentation with test results and version matrix

### Fixed
- Boost 1.89.0 compatibility patches applied to ur_rtde v1.6.0 source
  - Changed `boost::asio::io_service` → `boost::asio::io_context` in `rtde.h` and `rtde.cpp`
- Fixed DLL deployment in NuGet package (`runtimes/win-x64/native/` structure)
- Corrected P/Invoke calling conventions (Cdecl) for all native methods

### Technical Details
- **Native Libraries**:
  - `rtde.dll` (777 KB) - ur_rtde v1.6.0 core library
  - `ur_rtde_c_api.dll` (46.5 KB) - C API facade with 56 exported functions
- **Managed Assemblies**:
  - `UR.RTDE.dll` (21 KB) for net48 and net8.0
- **Dependencies**:
  - Boost 1.89.0 (113 packages via vcpkg)
  - Visual Studio 2022 (v143 toolset)
- **Build Time**: ~70 minutes total (60 min Boost, 10 min native)
- **Test Results**:
  - Connection: ✅ Stable
  - Streaming: ✅ 98.6 Hz (986 samples in 10s, C# overhead)
  - Movement: ✅ ±0.01 rad precision
  - Emergency stop: ✅ Immediate response
  - Reconnection: ✅ Stable

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
  - [TEST_PLAN.md](TEST_PLAN.md) - Testing strategy

### Technical Details
- Built on ur_rtde v1.6.0 by SDU Robotics
- Target platforms: Windows x64 (macOS arm64 planned)
- .NET targets: .NET Framework 4.8, .NET 8.0
- Architecture: Native C++ P/Invoke (zero Python dependency)

---

## Version History

| Version | Date | ur_rtde | Status | Key Features |
|---------|------|---------|--------|--------------|
| **2.0.0** | 2025-10-28 | v1.6.0 | ✅ Production Ready | 61 methods, URSim validated, complete docs |
| **1.0.0** | 2025-10-27 | v1.6.0 | 🔄 Initial Release | Basic control/receive, NuGet packaging |

---

## Upgrade Guide

### From 1.0.0 to 2.0.0

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
1. Update NuGet package: `dotnet add package UR.RTDE --version 2.0.0`
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

---

**Note**: See [UPDATING_URRTDE.md](UPDATING_URRTDE.md) for instructions on updating to newer versions of the ur_rtde library.
