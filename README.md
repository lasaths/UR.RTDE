# UR.RTDE — C# Native Wrapper for Universal Robots

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![.NET](https://img.shields.io/badge/.NET-4.8%20%7C%208.0-512BD4)](https://dotnet.microsoft.com/)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20macOS-lightgrey)](https://github.com/lasaths/UR.RTDE)
[![Build](https://img.shields.io/badge/Build-Success-brightgreen)](https://github.com/lasaths/UR.RTDE)

**Professional C# wrapper** for Universal Robots RTDE interface using **native C++ P/Invoke**. Zero external dependencies - everything included in the NuGet package.

Built on the battle-tested [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) C++ library (v1.6.0) by SDU Robotics.

## ⚠️ AI-Built & Liability Disclaimer

- Portions of this repository (code, scripts, and documentation) were authored with AI assistance and curated by the project maintainer.
- This software is provided “AS IS”, without warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular purpose, and noninfringement. See the [LICENSE](LICENSE) for full terms.
- The author(s) and contributors shall not be liable for any claim, damages, or other liability, whether in an action of contract, tort, or otherwise, arising from, out of, or in connection with the software or the use or other dealings in the software.
- Robotics Safety: You are solely responsible for safe deployment. Always test in simulation first (e.g., URSim), validate limits, and follow applicable safety standards (e.g., ISO 10218/TS 15066) and your organization’s safety procedures.
- Not affiliated with or endorsed by Universal Robots A/S or SDU Robotics. All trademarks are the property of their respective owners.

---

## 🎯 Features

### Core Capabilities (v1.1.0.0)
- ✅ **Native Performance** - Direct C++ via P/Invoke (500+ Hz streaming)
- ✅ **Zero Dependencies** - No Python, all native DLLs included in NuGet  
- ✅ **Rhino 7 & 8** - Works in both (.NET Framework 4.8 and .NET 8)
- ✅ **Cross-Platform** - Windows x64 ready, macOS arm64 pending
- ✅ **One-Click Install** - Single NuGet package, automatic deployment

### API Coverage (70+ Methods - All Major Features Implemented)
- ✅ **Movement Control** (15 methods) - MoveJ/L, SpeedJ/L, ServoJ/C/L, advanced stop modes
- ✅ **Force Control** (5 methods) - **NEW!** ForceMode, ZeroFtSensor, damping/gain adjustment
- ✅ **Jogging & Teaching** (4 methods) - **NEW!** JogStart/Stop, TeachMode, freedrive
- ✅ **Kinematics** (3 methods) - IK, FK, solution checking for path planning
- ✅ **Data Streaming** (20 methods) - Position, velocity, force, temperature, current, analog I/O
- ✅ **I/O Control** (6 methods) - Digital/analog outputs, speed slider
- ✅ **Safety Monitoring** (4 properties, 3 methods) - Protective/emergency stop, safety status, trigger protective stop
- ✅ **Status Queries** (3 properties) - Program running, robot steady, detailed status bits
- ✅ **Connection Management** - Robust connect/disconnect, reconnection, timeout handling

**Total**: 70+ methods across RTDEControl, RTDEReceive, and RTDEIO interfaces

---

## 🤝 Robotiq Gripper Support (URCap)

Two options are available when the Robotiq URCap is installed on the controller:

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

## 📦 Installation

### From Local Build
```bash
dotnet add package UR.RTDE --source C:\Users\lasaths\Github\UR.RTDE\nupkgs
```

### From NuGet.org (coming soon)
```bash
dotnet add package UR.RTDE
```

**That's it!** No configuration needed. Native DLLs deploy automatically to your bin folder.

---

## 🚀 Quick Start

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

## 🔨 Building from Source

**Status**: ✅ **Build verified and working**  
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
.\build-complete.bat  # Builds everything: Boost → ur_rtde → C API → C# → NuGet
```

**Option 2: Manual Steps**
```powershell
# 1. Install Boost via vcpkg
cd C:\vcpkg
vcpkg install boost:x64-windows

# 2. Build ur_rtde
cd C:\Users\lasaths\Github\UR.RTDE\build-native\ur_rtde\build
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

## 📚 Documentation

- **[Implementation Complete Report](IMPLEMENTATION_COMPLETE.md)** ⭐ **NEW!** Complete feature list and status
- **[Test Report](TEST_REPORT.md)** ⭐ Complete URSim validation results (22/22 tests passed)
- **[Build Success Report](BUILD_SUCCESS.md)** - Complete build details and achievements
- [Build Instructions](BUILD_INSTRUCTIONS.md) - Manual build steps
- [Updating ur_rtde](UPDATING_URRTDE.md) - Step-by-step guide for updating to newer ur_rtde versions
- [Changelog](CHANGELOG.md) - Version history and upgrade guide
- [Agent Instructions](AGENTS.md) - For AI agents/developers
- [Feature Coverage](FEATURE_COVERAGE.md) - Complete API coverage analysis
- [Test Plan](TEST_PLAN.md) - Comprehensive test strategy

### Test Flags
- `ROBOT_IP`: overrides the default robot/URSim IP (default: `localhost`).
- `ENABLE_ROBOTIQ_TESTS`: set to `true` to run Robotiq tests (requires URCap).
- `ENABLE_FT_TESTS`: set to `true` to run FT zeroing test (may be unsupported in URSim).

---

## 🏷️ Release

We use SemVer. This version is 1.1.0.0.

Steps to publish a GitHub Release (CI will attach the nupkg and use CHANGELOG as notes):

```powershell
# Ensure your working tree is clean and up to date
git pull origin main

# Tag the release (match csproj version)
git tag v1.1.0.0 -m "UR.RTDE 1.1.0.0"
git push origin v1.1.0.0

# GitHub Actions will create the Release and attach the nupkg
```

NuGet publish (manual):

```powershell
dotnet pack src/UR.RTDE -c Release -o nupkgs
dotnet nuget push nupkgs/UR.RTDE.1.1.0.0.nupkg -k <API_KEY> -s https://api.nuget.org/v3/index.json
```

---

## 📄 License

MIT License - see [LICENSE](LICENSE)

**Credits**: Built on [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) by SDU Robotics

---

**Install now:** `dotnet add package UR.RTDE` 🚀
