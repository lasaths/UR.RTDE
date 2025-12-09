# UR.RTDE - C# Native Wrapper for Universal Robots

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![.NET](https://img.shields.io/badge/.NET-4.8%20%7C%208.0-512BD4)](https://dotnet.microsoft.com/)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20macOS-lightgrey)](https://github.com/lasaths/UR.RTDE)
[![Build](https://img.shields.io/badge/Build-Success-brightgreen)](https://github.com/lasaths/UR.RTDE)
[![NuGet](https://img.shields.io/nuget/v/UR.RTDE.svg)](https://www.nuget.org/packages/UR.RTDE)

**Professional C# wrapper** for Universal Robots RTDE interface using **native C++ P/Invoke**. Zero external dependencies - everything included in the NuGet package.

Built on the battle-tested [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) C++ library (v1.6.0) by SDU Robotics.

## AI-Built & Liability Disclaimer

- Portions of this repository (code, scripts, and documentation) were authored with AI assistance and curated by the project maintainer.
- This software is provided AS IS, without warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular purpose, and noninfringement. See the [LICENSE](LICENSE) for full terms.
- The author(s) and contributors shall not be liable for any claim, damages, or other liability, whether in an action of contract, tort, or otherwise, arising from, out of, or in connection with the software or the use or other dealings in the software.
- Robotics Safety: You are solely responsible for safe deployment. Always test in simulation first (e.g., URSim), validate limits, and follow applicable safety standards (e.g., ISO 10218/TS 15066) and your organizations safety procedures.
- Not affiliated with or endorsed by Universal Robots A/S or SDU Robotics. All trademarks are the property of their respective owners.

---

## Target Features

### Core Capabilities (v1.1.0.0)
- [OK] **Native Performance** - Direct C++ via P/Invoke (500+ Hz streaming)
- [OK] **Zero Dependencies** - No Python, all native DLLs included in NuGet  
- [OK] **Rhino 7 & 8** - Works in both (.NET Framework 4.8 and .NET 8)
- [OK] **Cross-Platform** - Windows x64 ready, macOS arm64 pending
- [OK] **One-Click Install** - Single NuGet package, automatic deployment

### API Coverage (70+ Methods - All Major Features Implemented)
- [OK] **Movement Control** (15 methods) - MoveJ/L, SpeedJ/L, ServoJ/C/L, advanced stop modes
- [OK] **Force Control** (5 methods) - **NEW!** ForceMode, ZeroFtSensor, damping/gain adjustment
- [OK] **Jogging & Teaching** (4 methods) - **NEW!** JogStart/Stop, TeachMode, freedrive
- [OK] **Kinematics** (3 methods) - IK, FK, solution checking for path planning
- [OK] **Data Streaming** (20 methods) - Position, velocity, force, temperature, current, analog I/O
- [OK] **I/O Control** (6 methods) - Digital/analog outputs, speed slider
- [OK] **Safety Monitoring** (4 properties, 3 methods) - Protective/emergency stop, safety status, trigger protective stop
- [OK] **Status Queries** (3 properties) - Program running, robot steady, detailed status bits
- [OK] **Connection Management** - Robust connect/disconnect, reconnection, timeout handling

**Total**: 70+ methods across RTDEControl, RTDEReceive, and RTDEIO interfaces

---

## Status & Coverage (summary)
- **Production ready**: URSim e-Series 5.23.0 validation; 22/22 core + 6/6 advanced tests passing (MoveJ/L, ServoJ/L, SpeedJ/L, ForceMode, Jog, TeachMode, Protective Stop, extended receive data).
- **Performance**: Native P/Invoke, fast-path register bridge for Robotiq; sustained high-frequency streaming validated in URSim.
- **Coverage focus**: Movement, kinematics, safety, receive data (targets, force, temps, currents), digital/analog I/O, Robotiq (native, URScript, RTDE bridge).
- **Remaining**: Dashboard/Script client and extended receive extras can be added in future iterations; see AGENTS.md for roadmap notes.

---

## Robotiq Gripper Support (URCap)

Three options are available when the Robotiq URCap is installed on the controller:

- Option 1 (Native driver on 63352, no URScript dependency):
  - Use `RobotiqGripperNative` (wraps `ur_rtde::RobotiqGripper` socket client).
  - Example:
    ```csharp
    using var g = new RobotiqGripperNative("192.168.1.100");
    g.Connect();
    g.Activate();
    g.SetSpeed(0.5f);
    g.SetForce(0.5f);
    g.Open(moveMode: RobotiqMoveMode.WaitFinished);
    g.Close(moveMode: RobotiqMoveMode.WaitFinished);
    ```

- Option 2 (URScript client, simple):
  - Use `RobotiqGripper` over TCP port 30002 to call URCap functions (`rq_activate`, `rq_open`, `rq_close`, `rq_move`, `rq_set_speed`, `rq_set_force`).
  - Example:
    ```csharp
    var g = new RobotiqGripper("192.168.1.100", 30002);
    await g.ConnectAsync();
    await g.ActivateAsync();
    await g.SetSpeedAsync(128);
    await g.SetForceAsync(128);
    await g.CloseAsync();
    ```

- Option 3 (RTDE fast path, preferred):
  - Use `RobotiqGripperRtde` for low-latency commands via RTDE input/output registers.
  - A one-time URScript bridge is uploaded (requires URCap); then commands are issued by writing registers.
  - Example:
    ```csharp
    using var ctrl = new RTDEControl("192.168.1.100");
    using var recv = new RTDEReceive("192.168.1.100");
    var g = new RobotiqGripperRtde(ctrl, recv);
    await g.InstallBridgeAsync();
    await g.ActivateAsync();
    await g.CloseAsync();
    ```

Notes:
- The `rq_*` functions come from the Robotiq URCap. Without the URCap, these calls are undefined and tests should remain disabled.
- Default register mapping follows SDU examples: `input_int[0]`=command, `input_int[1]`=value, `output_int[0]`=status, `output_int[1]`=position.

---

## Package Installation

### From Local Build
```bash
dotnet add package UR.RTDE --source C:\path\to\UR.RTDE\nupkgs
```

### From NuGet.org (coming soon)
```bash
dotnet add package UR.RTDE
```

**That's it!** No configuration needed. Native DLLs deploy automatically to your bin folder.

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

**Status**: [OK] **Build verified and working**  
**See**: [BUILD_SUCCESS.md](BUILD_SUCCESS.md) for build details

### Prerequisites
- Visual Studio 2022 with "Desktop development with C++" workload
- vcpkg (automatically configured during build)
- CMake (included with VS 2022)
- ~7 GB disk space (for Boost dependencies)
- ~70 minutes build time (60 min for Boost, 10 min for ur_rtde)

### Quick Build

**Option 1: Automated** (recommended)
```powershell
cd UR.RTDE
.\build-complete.bat  # Builds everything: Boost -> ur_rtde -> C API -> C# -> NuGet
```

**Option 2: Manual Steps**
```powershell
# 1. Install Boost via vcpkg
cd C:\vcpkg
vcpkg install boost:x64-windows

# 2. Build ur_rtde
cd C:\path\to\UR.RTDE\build-native\ur_rtde\build
cmake --build . --config Release

# 3. Build C API facade
cd ..\..\native\facade\build
cmake --build . --config Release

# 4. Build C# wrapper
cd ..\..\..\
dotnet build src\UR.RTDE -c Release

# 5. Create NuGet package
dotnet pack src\UR.RTDE -c Release -o nupkgs
```

**Important**: ur_rtde v1.6.0 source has been patched for Boost 1.89.0 compatibility. See [BUILD_SUCCESS.md](BUILD_SUCCESS.md) for technical details.

See [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) for complete manual instructions.

---

## Documentation

- Test results and plan: summarized below (full historical reports removed for brevity)
- [Changelog](CHANGELOG.md) - Version history and upgrade guide
- [Agent Instructions](AGENTS.md) - For AI agents/developers
- [docs/quickstart.md](docs/quickstart.md) - Quick start for Rhino/URSim
- [docs/troubleshooting.md](docs/troubleshooting.md) - Common runtime issues
- [docs/version-matrix.md](docs/version-matrix.md) - Supported versions and RIDs

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

We use SemVer. This version is 1.1.0.0 (NuGet normalized as 1.1.0).

Steps to publish a GitHub Release (manual):

```powershell
# Ensure your working tree is clean and up to date
git pull origin main

# Tag the release (match csproj version)
git tag v1.1.0.0 -m "UR.RTDE 1.1.0.0"
git push origin v1.1.0.0

# Create a GitHub Release for tag v1.1.0.0 and upload the nupkg
```

NuGet publish (manual):

```powershell
dotnet pack src/UR.RTDE -c Release -o nupkgs
# NuGet normalizes trailing .0 segments, so the file is '1.1.0'
dotnet nuget push nupkgs/UR.RTDE.1.1.0.nupkg -k <API_KEY> -s https://api.nuget.org/v3/index.json
```

---

## License

MIT License - see [LICENSE](LICENSE)

**Credits**: Built on [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) by SDU Robotics

---

**Install now:** `dotnet add package UR.RTDE`
