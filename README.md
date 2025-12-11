# UR.RTDE - C# Native Wrapper for Universal Robots

[![License: MIT](https://opensource.org/licenses/MIT)](https://opensource.org/licenses/MIT)
[![.NET](https://img.shields.io/badge/.NET-4.8%20%7C%208.0-512BD4)](https://dotnet.microsoft.com/)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20macOS-lightgrey)](https://github.com/lasaths/UR.RTDE)
[![Build](https://img.shields.io/badge/Build-Success-brightgreen)](https://github.com/lasaths/UR.RTDE)
[![NuGet](https://img.shields.io/nuget/v/UR.RTDE.svg)](https://www.nuget.org/packages/UR.RTDE)

Native C# wrapper for Universal Robots RTDE using a C++ P/Invoke facade. No Python dependency; NuGet package contains managed assemblies and native binaries. Based on [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) v1.6.2.

## AI-Built & Liability Disclaimer

- Some content was generated with AI assistance and reviewed by the maintainer.
- Provided AS IS without warranty; see [LICENSE](LICENSE).
- User is responsible for safe deployment and compliance with relevant safety procedures; validate in simulation (e.g., URSim) before use.
- Not affiliated with Universal Robots A/S or SDU Robotics; trademarks belong to their owners.

---

## Target Features

### Core Capabilities (v1.2.0.0)
- Native C++ P/Invoke; sustained high-frequency streaming.
- No external dependencies; native DLLs included in NuGet.
- Supports Rhino 7 (.NET 4.8) and Rhino 8 (.NET 8).
- Windows x64 tested; macOS arm64 planned.

### API Coverage (70+ methods)
- Movement: MoveJ/L, SpeedJ/L, ServoJ/C/L, stop modes.
- Force: ForceMode, ZeroFtSensor, damping/gain controls.
- Jog/Teach: JogStart/Stop, TeachMode, freedrive.
- Kinematics: IK, FK, solution checking.
- Data: position, velocity, force, temperature, current, analog I/O.
- I/O: digital/analog outputs, speed slider.
- Safety: protective/emergency stop, safety status.
- Connection/status helpers.

---

## Status & Coverage
- Validated on URSim e-Series 5.23.0; 22/22 core and 6/6 advanced tests passing (MoveJ/L, ServoJ/L, SpeedJ/L, ForceMode, Jog, TeachMode, protective stop, extended receive data).
- Native P/Invoke path with Robotiq fast-path register bridge; sustained high-frequency streaming confirmed in URSim.
- Focus areas: movement, kinematics, safety, receive data, digital/analog I/O, Robotiq (native, URScript, RTDE bridge).
- Pending: dashboard/script client additions and extended receive extras; see `AGENTS.md`.

---

## Robotiq Gripper Support (URCap)

Requires Robotiq URCap on the controller.

- Native driver (port 63352): `RobotiqGripperNative` wraps `ur_rtde::RobotiqGripper`.
- URScript client (port 30002): `RobotiqGripper` issues `rq_*` calls.
- RTDE fast path (preferred): `RobotiqGripperRtde` uses RTDE registers; installs a one-time URScript bridge.

Default register mapping: `input_int[0]` command, `input_int[1]` value, `output_int[0]` status, `output_int[1]` position.

---

## Package Installation

### From Local Build
```bash
dotnet add package UR.RTDE --source C:\path\to\UR.RTDE\nupkgs
```

### From NuGet.org
```bash
dotnet add package UR.RTDE
```

Native DLLs copy automatically to the output folder.

---

## Quick Start

```csharp
using UR.RTDE;

// Connect
using var control = new RTDEControl("192.168.1.100");
using var receive = new RTDEReceive("192.168.1.100");

// Move robot
double[] homeQ = { 0, -1.57, 1.57, -1.57, -1.57, 0 };
control.MoveJ(homeQ, speed: 1.05, acceleration: 1.4);

// Read state
double[] q = receive.GetActualQ();
Console.WriteLine($"Joint 0: {q[0]:F4} rad");
```

---

## Building from Source

Build is verified; see `BUILD_SUCCESS.md` for details.

### Prerequisites
- Visual Studio 2026 (v145 toolset) with Desktop C++ workload
- vcpkg
- CMake (bundled with VS 2026)
- ~7 GB disk for Boost; ~70 minutes initial build

### Quick Build

Automated:
```powershell
cd UR.RTDE
.\build-complete.bat
```

Manual:
```powershell
# Boost
cd C:\vcpkg
vcpkg install boost:x64-windows

# ur_rtde
cd C:\path\to\UR.RTDE\build-native\ur_rtde\build
cmake --build . --config Release

# C API facade
cd ..\..\native\facade\build
cmake --build . --config Release

# C# wrapper
cd ..\..\..\
dotnet build src\UR.RTDE -c Release

# NuGet
dotnet pack src\UR.RTDE -c Release -o nupkgs
```

ur_rtde v1.6.2 source includes Boost 1.89.0 compatibility patches; see `BUILD_SUCCESS.md` and `BUILD_INSTRUCTIONS.md`.

---

## Documentation

- Test results and plan summary below.
- `CHANGELOG.md` - Version history and upgrades.
- `AGENTS.md` - Agent instructions and roadmap.
- `docs/quickstart.md` - Rhino/URSim quick start.
- `docs/troubleshooting.md` - Runtime issues.
- `docs/version-matrix.md` - Supported versions and RIDs.

### Test Flags
- `ROBOT_IP`: overrides the default robot/URSim IP (default: `localhost`).
- `ENABLE_ROBOTIQ_TESTS`: set to `true` to run Robotiq tests (requires URCap).
- `ENABLE_FT_TESTS`: set to `true` to run FT zeroing test (may be unsupported in URSim).

---

## Test Plan (summary)
- Location: `samples/ExtendedFeaturesTest/Program.cs`
- Coverage: kinematics, extended data, safety status, analog/digital I/O, advanced motion (ServoC/ServoStop/SpeedStop), speed slider.
- Success criteria: 7/7 categories pass; IK round-trip error < 0.001 rad; safety flags valid; I/O round-trips match commands; robot stops cleanly.
- Execution: `dotnet run -c Release` from `samples/ExtendedFeaturesTest` (optional test name filter).

## Latest Test Results (URSim snapshot)
- Date/Platform: 2025-10-27, Windows 11, .NET 8.0, URSim e-Series 5.23.0 (Docker, localhost).
- Outcome: 7/7 tests passed; average streaming 98.6 Hz; MoveJ precision +/-0.01 rad; emergency stop immediate; reconnection stable.

## Update ur_rtde (quick guide)
1) Check new upstream tag/release notes on gitlab.com/sdurobotics/ur_rtde.  
2) Refresh `build-native/ur_rtde` to the tag (backup old copy); apply Boost/compat patches if still needed.  
3) Rebuild native lib and facade; copy DLLs into `src/UR.RTDE/runtimes/{rid}/native/`.  
4) Add any new C API exports, P/Invoke bindings, and wrapper methods; update docs/tests.  
5) Run test suite (URSim), bump versions, refresh README/AGENTS/CHANGELOG, and pack/publish NuGet.

---

## Release

We use SemVer. This version is 1.2.0.0 (NuGet normalized as 1.2.0).

Steps to publish a GitHub Release (manual):

```powershell
# Ensure your working tree is clean and up to date
git pull origin main

# Tag the release (match csproj version)
git tag v1.2.0.0 -m "UR.RTDE 1.2.0.0"
git push origin v1.2.0.0

# Create a GitHub Release for tag v1.2.0.0 and upload the nupkg
```

NuGet publish (manual):

```powershell
dotnet pack src/UR.RTDE -c Release -o nupkgs
# NuGet normalizes trailing .0 segments, so the file is '1.2.0'
dotnet nuget push nupkgs/UR.RTDE.1.2.0.nupkg -k <API_KEY> -s https://api.nuget.org/v3/index.json
```

---

## License

MIT License - see [LICENSE](LICENSE)

**Credits**: Built on [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) by SDU Robotics

---

**Install now:** `dotnet add package UR.RTDE`
