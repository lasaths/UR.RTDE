# UR.RTDE - C# Native Wrapper for Universal Robots

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![.NET](https://img.shields.io/badge/.NET-4.8%20%7C%208.0-512BD4)](https://dotnet.microsoft.com/)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20macOS-lightgrey)](https://github.com/lasaths/UR.RTDE)
[![Build](https://img.shields.io/badge/Build-Success-brightgreen)](https://github.com/lasaths/UR.RTDE)
[![NuGet](https://img.shields.io/nuget/v/UR.RTDE.svg)](https://www.nuget.org/packages/UR.RTDE)

Native C# wrapper for Universal Robots RTDE using a C++ P/Invoke facade. No Python dependency; NuGet package contains managed assemblies and native binaries. Based on [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) 1.6.3 (pinned commit `68ac4e1`).

## AI-Built & Liability Disclaimer

- Some content was generated with AI assistance and reviewed by the maintainer.
- Provided AS IS without warranty; see [LICENSE](LICENSE).
- User is responsible for safe deployment and compliance with relevant safety procedures; validate in simulation (e.g., URSim) before use.
- Not affiliated with Universal Robots A/S or SDU Robotics; trademarks belong to their owners.

---

## Target Features

### Core Capabilities (v1.6.3.9)
- Native C++ P/Invoke; sustained high-frequency streaming.
- No external dependencies; native libraries included in NuGet (`win-x64` DLLs, `osx-arm64` dylibs).
- Supports Rhino 7 (.NET 4.8) and Rhino 8 (.NET 8).
- Windows x64 and macOS arm64 (Apple Silicon) native runtimes included in NuGet (`win-x64`, `osx-arm64`); macOS hardened for Rhino 8 in-process loading (v1.6.3.9+).

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
- macOS Rhino 8: package v1.6.3.9+ contains the thread-safe native bootstrap and a single hidden-symbol `osx-arm64` C facade dylib used to avoid Rhino Boost collisions. After updating a Grasshopper plugin, remove stale `librtde*.dylib`/Boost dylibs from the deploy folder, then fully quit and restart Rhino before testing Connect.
- Pending: dashboard/script client additions, extended receive extras, and end-to-end Rhino 8 Connect confirmation on the target macOS machine; see `AGENTS.md`.

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

Native libraries copy automatically to the output folder (`runtimes/{rid}/native/`).

---

## URSim via Docker

This project is validated against **URSim e-Series running in Docker**. For local development and testing, this is the recommended setup.

### 1. Install Docker Desktop

Install Docker Desktop for Windows or macOS and make sure `docker` works from your terminal.

### 2. Start URSim (recommended: Docker Compose)

From this repository root:

```bash
docker compose -f docker/ursim/docker-compose.yml up -d
```

See [docker/ursim/README.md](docker/ursim/README.md) for port list, optional External Control image, and troubleshooting.

Published ports (bound to `127.0.0.1` on the host):
- `6080`: browser-based VNC UI
- `5900`: VNC client
- `29999`: dashboard server
- `30001`–`30004`: robot interfaces (**`30003`** = RTDE control, **`30004`** = RTDE receive)
- `30002`: URScript client
- `50002`: External Control URCap (mapped; service may be inactive until URCap is installed)

Verify RTDE control is reachable (required for `RTDEControl` / MoveJ):

```bash
nc -zv 127.0.0.1 30003
nc -zv 127.0.0.1 30004
```

### 3. Open the simulator UI

Open [http://127.0.0.1:6080/vnc.html?host=localhost&port=6080](http://127.0.0.1:6080/vnc.html?host=localhost&port=6080).

Inside URSim:
- Power on the robot
- Release brakes
- Clear any startup or safety dialogs before running tests or motion commands

### 4. Connect from `UR.RTDE`

Use `127.0.0.1` or `localhost` when URSim is running with the port mapping above.

```bash
# macOS / Linux
nc -zv 127.0.0.1 30003
nc -zv 127.0.0.1 30004
```

```powershell
# Windows
Test-NetConnection 127.0.0.1 -Port 30003
Test-NetConnection 127.0.0.1 -Port 30004
```

Notes:
- For a real robot, replace `127.0.0.1` with the robot IP on your network.
- **Receive-only on port 30004 is not enough** for motion: `RTDEControl` needs **30003** published to the host.
- Optional External Control URCap: `docker compose -f docker/ursim/docker-compose.yml -f docker/ursim/docker-compose.external-control.yml up -d --build`

---

## Quick Start

```csharp
using UR.RTDE;

// Use "localhost" for URSim in Docker, or the robot IP for hardware.
using var control = new RTDEControl("localhost");
using var receive = new RTDEReceive("localhost");

// Move robot
double[] homeQ = { 0, -1.57, 1.57, -1.57, -1.57, 0 };
control.MoveJ(homeQ, speed: 1.05, acceleration: 1.4);

// Read state
double[] q = receive.GetActualQ();
Console.WriteLine($"Joint 0: {q[0]:F4} rad");
```

---

## Building from Source

Build is verified on Windows; see `native/BUILD.md` for Windows and macOS instructions.

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

Windows builds use ur_rtde v1.6.x with Boost via vcpkg. macOS arm64 builds use Boost 1.85 (`brew install boost@1.85`); see `native/BUILD.md`.

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
- Location: `samples/URSimTests/Program.cs` (core) and `samples/URSimTests/AdvancedTests.cs`
- Coverage: connection, streaming, MoveJ, emergency stop, kinematics, extended data, safety, I/O, advanced motion.
- Success criteria: core and advanced suites pass against URSim; see `samples/URSimTests/README.md`.
- Execution: `dotnet run -c Release --project samples/URSimTests` (optional test filter argument).

## Latest Test Results (URSim snapshot)
- Date/Platform: 2025-10-27, Windows 11, .NET 8.0, URSim e-Series 5.23.0 (Docker, localhost).
- Outcome: 7/7 tests passed; average streaming 98.6 Hz; MoveJ precision +/-0.01 rad; emergency stop immediate; reconnection stable.

## Update ur_rtde (quick guide)
1) Check new upstream tag/release notes on gitlab.com/sdurobotics/ur_rtde.  
2) Refresh `build-native/ur_rtde` to the exact pinned commit hash (not a moving branch/tag); backup old copy and apply Boost/compat patches if still needed.  
3) Rebuild native lib and facade; copy DLLs into `src/UR.RTDE/runtimes/{rid}/native/`.  
4) Add any new C API exports, P/Invoke bindings, and wrapper methods; update docs/tests.  
5) Run test suite (URSim), bump versions, refresh README/AGENTS/CHANGELOG, and pack/publish NuGet.

---

## Release

**Versioning:** package version tracks bundled **ur_rtde** (e.g. `1.6.3.0` = ur_rtde 1.6.3; fourth segment is for wrapper-only fixes). Current release is **1.6.3.9**.

**NuGet readme:** badge images use [trusted domains](https://learn.microsoft.com/nuget/nuget-org/package-readme-on-nuget-org#allowed-domains-for-images-and-badges) only (`img.shields.io`).

Steps to publish a GitHub Release (manual):

```powershell
# Ensure your working tree is clean and up to date
git pull origin master

# Tag the release (match UR.RTDE.csproj Version)
git tag v1.6.3.9 -m "UR.RTDE 1.6.3.9 (ur_rtde 1.6.3)"
git push origin v1.6.3.9

# Create a GitHub Release for tag v1.6.3.9 and upload the nupkg
```

NuGet publish (manual):

```powershell
dotnet pack src/UR.RTDE -c Release -o nupkgs
dotnet nuget push nupkgs/UR.RTDE.1.6.3.9.nupkg -k <API_KEY> -s https://api.nuget.org/v3/index.json
```

---

## License

MIT License - see [LICENSE](LICENSE)

**Credits**: Built on [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) by SDU Robotics

---

**Install now:** `dotnet add package UR.RTDE`
