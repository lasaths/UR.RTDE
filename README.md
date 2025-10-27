# UR.RTDE — C# Native Wrapper for Universal Robots

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![.NET](https://img.shields.io/badge/.NET-4.8%20%7C%208.0-512BD4)](https://dotnet.microsoft.com/)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20macOS-lightgrey)](https://github.com/lasaths/UR.RTDE)

**Professional C# wrapper** for Universal Robots RTDE interface using **native C++ P/Invoke**. Zero external dependencies - everything included in the NuGet package.

Built on the battle-tested [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) C++ library (v1.6.0) by SDU Robotics.

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

```bash
dotnet add package UR.RTDE
```

**That's it!** No configuration needed. Native DLLs deploy automatically.

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

**Prerequisites**: Visual Studio 2022 with "Desktop development with C++" workload

```bat
# Open Developer Command Prompt for VS 2022
cd UR.RTDE
build-native.bat
```

See [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) for details.

---

## 📚 Documentation

- [Quick Start Guide](docs/quickstart.md)
- [API Reference](docs/api-reference.md)  
- [Build Instructions](BUILD_INSTRUCTIONS.md)
- [Troubleshooting](docs/troubleshooting.md)

---

## 📄 License

MIT License - see [LICENSE](LICENSE)

**Credits**: Built on [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) by SDU Robotics

---

**Install now:** `dotnet add package UR.RTDE` 🚀
