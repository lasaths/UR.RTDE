# URSim Integration Tests

This project contains comprehensive integration tests for UR.RTDE against URSim.

## Prerequisites

1. **URSim running** at `172.18.0.2`
   - Use the provided Docker container
   - Access VNC interface: http://172.18.0.2:6080/vnc.html

2. **Robot initialized**
   - Power on the virtual robot
   - Release brakes
   - Ensure no errors in URSim

3. **Native DLLs prepared**
   - Run `setup-tests.bat` from repository root
   - This copies Boost thread DLL and native libraries
   - Required: `boost_thread-vc143-mt-x64-1_89.dll` from vcpkg

## Running the Tests

### First Time Setup
```bash
# From repository root
.\setup-tests.bat
```

### Run Tests
```bash
cd samples/URSimTests
dotnet run -c Release
```

## Test Suite

### Test 1: Basic Connection
- Verifies connection to control interface
- Verifies connection to receive interface
- Checks connection stability

### Test 2: Receive Interface
- Reads actual joint positions (6 values)
- Reads actual TCP pose (6 values: x, y, z, rx, ry, rz)
- Reads robot mode
- Reads safety mode

### Test 3: Control Interface
- Gets current TCP pose
- Gets current joint positions
- Validates data (no NaN/infinity)

### Test 4: Streaming Data
- Streams joint positions for 10 seconds
- Measures actual frequency
- Validates frequency > 100 Hz
- Reports statistics

### Test 5: Joint Movement
- Reads current position
- Moves joint 0 by 0.1 radians
- Monitors movement completion
- Validates final position

### Test 6: Emergency Stop
- Starts long movement
- Triggers emergency stop mid-movement
- Validates robot stopped

### Test 7: Reconnection
- Connects to robot
- Disconnects
- Reconnects with new instance
- Validates data reading after reconnection

## Expected Output

```
═══════════════════════════════════════════════
  UR.RTDE Integration Tests - URSim
═══════════════════════════════════════════════
Target: 172.18.0.2
Time: 2025-10-27 21:30:00

Running: Test 1: Basic Connection
─────────────────────────────────────────────
  Connected to control interface
  Connected to receive interface
  Connection stable
✓ PASSED: Test 1: Basic Connection

[... more tests ...]

═══════════════════════════════════════════════
  Test Summary
═══════════════════════════════════════════════
Passed: 7
Failed: 0
Total:  7

✓ ALL TESTS PASSED!
```

## Troubleshooting

### Connection Refused
- Ensure URSim is running: `docker ps`
- Check URSim logs
- Verify port 30004 is accessible: `Test-NetConnection 172.18.0.2 -Port 30004`

### Robot Not Moving
- Check URSim VNC interface
- Ensure robot is powered on
- Release brakes in URSim
- Check for safety stops

### Streaming Frequency Low
- Check CPU usage
- Ensure no other programs using RTDE
- Verify network latency to Docker container

## Performance Benchmarks

Expected results on typical development machine:

- Connection time: < 1 second
- Streaming frequency: 400-500 Hz
- MoveJ response time: < 100 ms
- Emergency stop response: < 200 ms

## Notes

- Tests use conservative movement parameters (speed: 0.5, acceleration: 0.5)
- All movements are small (< 0.5 radians) for safety
- Emergency stop uses 5 rad/s² deceleration
- Each test has 30-second timeout
