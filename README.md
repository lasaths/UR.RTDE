# UR.RTDE — Universal Robots RTDE Wrapper

A **C# wrapper** for the [SDU Robotics `ur_rtde`](https://gitlab.com/sdurobotics/ur_rtde) C++ library (v1.6.2), designed for **Rhino 7** (.NET Framework 4.8) and **Rhino 8** (.NET 8) on **Windows x64** and **macOS arm64**.

## Features

- **Control Interface**: MoveJ, MoveL, SpeedJ/L, StopJ/L, SetTcp, SetPayload, watchdog
- **Receive Interface**: ActualQ, ActualTcpPose, RobotMode, SafetyMode, RuntimeState, digital IO
- **Multi-platform**: Windows x64, macOS arm64 (via RID-specific native binaries)
- **Multi-framework**: .NET Framework 4.8 and .NET 8
- **NuGet distribution**: Single package with embedded native dependencies

## Quick Start

```bash
# Install via NuGet (in Grasshopper plugin project)
dotnet add package UR.RTDE
```

```csharp
using UR.RTDE;

// Connect to robot
using var control = new RTDEControl("192.168.1.100");
control.Connect();

// Move to home position
double[] homeQ = { 0, -90, 90, -90, -90, 0 }; // degrees
control.MoveJ(homeQ);

// Stop
control.StopJ();
```

## Platform Support

| Platform       | .NET Target | RID         | Status |
|----------------|-------------|-------------|--------|
| Rhino 7 Win    | net48       | win-x64     | ✅     |
| Rhino 8 Win    | net8.0      | win-x64     | ✅     |
| Rhino 8 macOS  | net8.0      | osx-arm64   | ✅     |

## Documentation

- [Quickstart Guide](docs/quickstart.md)
- [Troubleshooting](docs/troubleshooting.md)
- [API Reference](docs/api-reference.md)
- [Version Matrix](docs/version-matrix.md)

## Development

See [AGENTS.md](AGENTS.md) for the software engineering agent prompt and work plan.

## License

This wrapper is provided under MIT License. The underlying `ur_rtde` library is licensed under its own terms (see `native/ur_rtde/LICENSE`).

## Safety Notice

⚠️ **Always test robot movements in a safe environment.** Use proper safety stops and ensure emergency stops are accessible.
