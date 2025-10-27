# UR.RTDE — C# Native Wrapper for Universal Robots

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![.NET](https://img.shields.io/badge/.NET-4.8%20%7C%208.0-512BD4)](https://dotnet.microsoft.com/)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20macOS-lightgrey)](https://github.com/lasaths/UR.RTDE)
[![Build](https://img.shields.io/badge/Build-Success-brightgreen)](https://github.com/lasaths/UR.RTDE)

**Professional C# wrapper** for Universal Robots RTDE interface using **native C++ P/Invoke**. Zero external dependencies - everything included in the NuGet package.

Built on the battle-tested [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) C++ library (v1.6.0) by SDU Robotics.

> **✅ Status**: Build complete! Windows x64 native binaries ready. See [BUILD_SUCCESS.md](BUILD_SUCCESS.md) for details.

---

## 🎯 Features

- ✅ **Native Performance** - Direct C++ via P/Invoke (500+ Hz streaming)
- ✅ **Zero Dependencies** - No Python, all native DLLs included in NuGet  
- ✅ **Rhino 7 & 8** - Works in both (.NET Framework 4.8 and .NET 8)
- ✅ **Cross-Platform** - Windows x64, macOS arm64
- ✅ **One-Click Install** - Single NuGet package, automatic deployment
- ✅ **Production Ready** - Validated with URSim e-Series 5.23.0

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

- **[Build Success Report](BUILD_SUCCESS.md)** ⭐ Complete build details and achievements
- [Build Instructions](BUILD_INSTRUCTIONS.md) - Manual build steps
- [Agent Instructions](AGENTS.md) - For AI agents/developers
- [Quick Start Guide](docs/quickstart.md) (coming soon)
- [API Reference](docs/api-reference.md) (coming soon)
- [Troubleshooting](docs/troubleshooting.md) (coming soon)

---

## 📄 License

MIT License - see [LICENSE](LICENSE)

**Credits**: Built on [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) by SDU Robotics

---

**Install now:** `dotnet add package UR.RTDE` 🚀
