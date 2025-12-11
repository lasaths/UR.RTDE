# Quick Start Guide

## Installation

### NuGet Package (Recommended)
```bash
# In your Grasshopper plugin project
dotnet add package UR.RTDE
```

### Manual Build
See [Building from Source](../native/BUILD.md)

## Basic Usage

### 1. Connect and Move Robot

```csharp
using UR.RTDE;

// Connect to robot
using var control = new RTDEControl("192.168.1.100");

// Move to joint position (radians)
double[] homeQ = { 0, -1.57, 1.57, -1.57, -1.57, 0 };
control.MoveJ(homeQ, speed: 1.05, acceleration: 1.4);

// Stop
control.StopJ();
```

### 2. Stream Joint Data

```csharp
using UR.RTDE;

// Create receive interface
using var receive = new RTDEReceive("192.168.1.100");

// Subscribe to state updates
receive.StateReceived += (sender, state) =>
{
    Console.WriteLine($"J0: {state.ActualQ[0]:F4} rad");
};

// Start streaming at 500 Hz (default)
receive.StartReceiving();

// ... do work ...

// Stop streaming
receive.StopReceiving();
```

### 3. Read Current State

```csharp
using var receive = new RTDEReceive("192.168.1.100");

// Get current joint positions
double[] q = receive.GetActualQ();

// Get TCP pose
double[] pose = receive.GetActualTcpPose();

// Get robot mode
int mode = receive.GetRobotMode();
```

## Rhino 7 (Windows .NET 4.8)

### Grasshopper Component Example

```csharp
using System;
using Grasshopper.Kernel;
using UR.RTDE;

public class URConnectComponent : GH_Component
{
    private RTDEControl? _control;

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        string ip = "";
        bool connect = false;

        DA.GetData(0, ref ip);
        DA.GetData(1, ref connect);

        if (connect && _control == null)
        {
            _control = new RTDEControl(ip);
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, 
                "Connected to " + ip);
        }
        else if (!connect && _control != null)
        {
            _control.Dispose();
            _control = null;
        }

        DA.SetData(0, _control?.IsConnected ?? false);
    }
}
```

## Rhino 8 (Windows/macOS .NET 8)

Same API, works cross-platform. NuGet automatically includes:
- `win-x64` native library for Windows
- `osx-arm64` native library for macOS Apple Silicon

## Error Handling

```csharp
try
{
    using var control = new RTDEControl("192.168.1.100");
    control.MoveJ(targetQ);
}
catch (RTDEConnectionException ex)
{
    // Connection failed or lost
    Console.WriteLine($"Connection error: {ex.Message}");
}
catch (RTDEException ex)
{
    // Command failed
    Console.WriteLine($"Command error: {ex.Message}");
}
```

## Safety Notes

[WARN] **IMPORTANT**
- Always test in a safe environment
- Keep emergency stop accessible
- Use proper safety boundaries
- Start with low speeds and accelerations
- Never block the UI thread with synchronous moves

## Threading

- `RTDEControl`: Methods are blocking by default. Use `asynchronous: true` for non-blocking moves.
- `RTDEReceive`: Uses background thread for streaming. Events are marshaled to calling thread.

## Performance Tips

1. **Streaming frequency**: Default RTDE rate is 500 Hz (e-Series). Adjust `updateRateMs` in `StartReceiving()` if needed.
2. **Avoid UI blocking**: Use `asynchronous: true` for moves, or run on background thread.
3. **Connection reuse**: Keep `RTDEControl`/`RTDEReceive` alive for multiple operations.

## Next Steps

- [API Reference](api-reference.md)
- [Troubleshooting](troubleshooting.md)
- [Sample Projects](../samples/)
- [Version Matrix](version-matrix.md)

## Robotiq Gripper (URCap)

If the Robotiq URCap is installed on the controller:

- Quick (URScript client):
```csharp
var g = new UR.RTDE.RobotiqGripper("192.168.1.100", 30002);
await g.ConnectAsync();
await g.ActivateAsync();
await g.CloseAsync();
```

- Fast (RTDE registers):
```csharp
using var ctrl = new UR.RTDE.RTDEControl("192.168.1.100");
using var recv = new UR.RTDE.RTDEReceive("192.168.1.100");
using var io = new UR.RTDE.RTDEIO("192.168.1.100");
var g = new UR.RTDE.RobotiqGripperRtde(ctrl, recv, io);
await g.InstallBridgeAsync();
await g.CloseAsync();
```

Test flags:
- `ENABLE_ROBOTIQ_TESTS=true` to run Robotiq tests
- `ENABLE_FT_TESTS=true` to run FT zeroing test
