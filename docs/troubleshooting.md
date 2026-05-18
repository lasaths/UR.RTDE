# Troubleshooting

## Native Library Load Issues

### Windows

**Symptom**: `DllNotFoundException: Unable to load DLL 'ur_rtde_c_api'`

**Solutions**:
1. **Check native library location**: NuGet should place it in `bin\Debug\net48\runtimes\win-x64\native\`
2. **Verify dependencies**: Ensure Boost DLLs are in the same directory or on PATH
3. **Use Process Monitor**: Download [Process Monitor](https://docs.microsoft.com/en-us/sysinternals/downloads/procmon) to see where .NET is looking for DLLs

**Manual fix**:
```bash
# Copy native DLLs to output directory
copy runtimes\win-x64\native\*.dll bin\Debug\net48\
```

### macOS

**Symptom**: `DllNotFoundException: Unable to load library 'ur_rtde_c_api'`

**Solutions**:
1. **Gatekeeper**: macOS blocks unsigned dylibs
   ```bash
   # Remove quarantine flag
   xattr -d com.apple.quarantine libur_rtde_c_api.dylib
   ```

2. **Check dylib location**: Should be in `bin/Debug/net8.0/runtimes/osx-arm64/native/`, or in the Grasshopper plugin folder beside `UR.RTDE.dll` with the same `runtimes/osx-arm64/native/` tree preserved.

3. **Verify architecture**: Rhino 8 macOS is arm64
   ```bash
   file libur_rtde_c_api.dylib
   # Should show: Mach-O 64-bit dynamically linked shared library arm64
   ```

4. **Check dependencies**:
   ```bash
   otool -L libur_rtde_c_api.dylib
   # For UR.RTDE 1.6.3.9+, this should list only libur_rtde_c_api itself,
   # libc++, and libSystem. It should not list librtde or Boost dylibs.
   ```

**Manual fix**:
```bash
# Copy to output directory
cp -r runtimes/osx-arm64/native/*.dylib bin/Debug/net8.0/
```

### Rhino 8 / Grasshopper crash on macOS arm64 during Connect

**Symptom**: Rhino aborts on `UR Session -> Connect`, often with a native stack inside Boost.Asio `deadline_timer_service` during `ur_rtde_receive_create`.

**Known fixes in UR.RTDE 1.6.3.9+**:
1. macOS bootstrap is thread-safe before `RTDEControl`, `RTDEReceive`, `RTDEIO`, or `RobotiqGripperNative` enters P/Invoke.
2. macOS native loading uses `RTLD_LOCAL` and `RTLD_FIRST` for the C facade.
3. The macOS arm64 runtime is a single `libur_rtde_c_api.dylib` with `ur_rtde` and Boost linked internally.
4. Separate `librtde*.dylib` and Boost dylibs are not part of the package and should be removed from Rhino deploy folders.

**Checklist**:
1. Fully quit Rhino after replacing any `.gha`, `UR.RTDE.dll`, or `.dylib` files. macOS and .NET will not unload the old native library from a running Rhino process.
2. Use a single deploy folder under:
   `~/Library/Application Support/McNeel/Rhinoceros/8.0/Plug-ins/Grasshopper (...044e8580d9cf...)/Libraries/UR.RTDE.Grasshopper/`
3. Remove duplicate copies from wrong Grasshopper GUID folders, `Documents`, old Yak/manual drops, or prior test deploy folders.
4. Preserve the NuGet runtime tree:
   `UR.RTDE.Grasshopper/runtimes/osx-arm64/native/libur_rtde_c_api.dylib`.
   Remove stale `librtde*.dylib` and `libboost*.dylib` files from the same deploy tree.
5. Verify the deployed runtime:
   ```bash
   cd "$HOME/Library/Application Support/McNeel/Rhinoceros/8.0/Plug-ins/Grasshopper (...044e8580d9cf...)/Libraries/UR.RTDE.Grasshopper/runtimes/osx-arm64/native"
   ls -lh libur_rtde_c_api.dylib
   otool -L libur_rtde_c_api.dylib
   find ../.. -name 'librtde*.dylib' -o -name 'libboost*.dylib'
   nm -gU libur_rtde_c_api.dylib | c++filt | grep -E 'boost::|ur_rtde::|std::__' || true
   ```

Rhino can run from a non-standard volume path. UR.RTDE does not depend on `/Applications`; what matters is the plugin folder that contains `UR.RTDE.dll` and its native runtime assets.

## Connection Issues

### Cannot connect to robot

**Symptom**: `RTDEConnectionException: Failed to connect to robot`

**Checklist**:
1. [OK] Robot is powered on
2. [OK] Robot is on same network as computer
3. [OK] Ping robot IP: `ping 192.168.1.100`
4. [OK] Port 30004 is open (RTDE receive)
5. [OK] Port 30003 is open (RTDE control — required for `RTDEControl`, MoveJ, etc.)
6. [OK] Firewall allows connection
7. [OK] Robot is not in "Local" mode for hardware (URSim Docker can use Local Control with bypass flags)

**Test connection**:
```bash
# Windows
Test-NetConnection -ComputerName 192.168.1.100 -Port 30004

# macOS/Linux
nc -zv 192.168.1.100 30003
nc -zv 192.168.1.100 30004
```

**URSim Docker**: publish `30003` and `30004` to the host. From the repo root:

```bash
docker compose -f docker/ursim/docker-compose.yml up -d
nc -zv 127.0.0.1 30003
```

### Connection drops during operation

**Symptom**: `RTDEConnectionException: Connection error`

**Solutions**:
1. **Network stability**: Check for packet loss (`ping -t 192.168.1.100`)
2. **Watchdog timeout**: Call `control.TriggerWatchdog()` regularly (every 1 second)
3. **Use reconnect**: 
   ```csharp
   if (!control.IsConnected)
   {
       control.Reconnect();
   }
   ```

## Robot Not Moving

### MoveJ/MoveL returns but robot doesn't move

**Checklist**:
1. [OK] Robot is in "Remote Control" mode (not "Local")
2. [OK] No protective stop active (check teach pendant)
3. [OK] No safety violations (check SafetyMode)
4. [OK] Target position is reachable (within joint limits)
5. [OK] Speed/acceleration are within limits

**Debug**:
```csharp
var receive = new RTDEReceive("192.168.1.100");
Console.WriteLine($"Robot Mode: {receive.GetRobotMode()}");
Console.WriteLine($"Safety Mode: {receive.GetSafetyMode()}");
Console.WriteLine($"Runtime State: {receive.GetRuntimeState()}");
```

**Robot Mode Codes** (reference):
- `-1`: `ROBOT_MODE_DISCONNECTED`
- `0`: `ROBOT_MODE_CONFIRM_SAFETY`
- `1`: `ROBOT_MODE_BOOTING`
- `2`: `ROBOT_MODE_POWER_OFF`
- `3`: `ROBOT_MODE_POWER_ON`
- `4`: `ROBOT_MODE_IDLE`
- `5`: `ROBOT_MODE_BACKDRIVE`
- `6`: `ROBOT_MODE_RUNNING`
- `7`: `ROBOT_MODE_UPDATING_FIRMWARE`

### Asynchronous moves don't complete

**Issue**: Calling `MoveJ(..., asynchronous: true)` multiple times too quickly

**Solution**: Wait for previous move to complete or use `StopJ()`/`StopL()` before next move
```csharp
control.MoveJ(q1, asynchronous: true);
Thread.Sleep(100); // Give move time to start
// ... or poll for completion
control.MoveJ(q2, asynchronous: true);
```

## Performance Issues

### UI freezing in Grasshopper

**Cause**: Blocking synchronous moves on UI thread

**Solution**: Use async moves or background thread
```csharp
// Option 1: Async move
control.MoveJ(target, asynchronous: true);

// Option 2: Background thread
Task.Run(() => control.MoveJ(target));
```

### Streaming rate too slow

**Cause**: Default update rate may be throttled

**Solution**: Adjust `updateRateMs` parameter
```csharp
// Stream at full RTDE rate (500 Hz = 2ms)
receive.StartReceiving(updateRateMs: 2);
```

## Build Issues

### CMake cannot find ur_rtde

**Error**: `Could not find a package configuration file provided by "ur_rtde"`

**Solution**: Specify path to ur_rtde install
```bash
cmake .. -Dur_rtde_DIR=/path/to/ur_rtde/install/lib/cmake/ur_rtde
```

### Boost not found

**Windows**:
```bash
vcpkg install boost-system:x64-windows boost-thread:x64-windows
cmake .. -DCMAKE_TOOLCHAIN_FILE=[vcpkg]/scripts/buildsystems/vcpkg.cmake
```

**macOS**:
```bash
brew install boost
```

## Still Having Issues?

1. Check [ur_rtde GitLab issues](https://gitlab.com/sdurobotics/ur_rtde/-/issues)
2. Enable verbose logging:
   ```csharp
   var control = new RTDEControl("192.168.1.100", 
       flags: (ushort)NativeMethods.Flags.Verbose);
   ```
3. Test with URSim (UR robot simulator)
4. Verify UR PolyScope version compatibility (see [version-matrix.md](version-matrix.md))

## Robotiq URCap / Gripper Notes

### `rq_*` not defined / script errors
- Symptom: URScript functions like `rq_activate()` are undefined.
- Cause: Robotiq URCap not installed/enabled on the controller/URSim.
- Fix: Install the Robotiq URCap and enable it in PolyScope. Until then, keep `ENABLE_ROBOTIQ_TESTS` disabled.

### RTDE-register gripper bridge not responding
- Ensure the bridge script was uploaded (`RobotiqGripperRtde.InstallBridgeAsync`).
- Confirm default register mapping is free: `input_int[0..1]`, `output_int[0..1]`.
- Check controller is in Remote mode and running; no protective/emergency stop.

### FT zeroing fails in URSim
- `ZeroFtSensor()` can return `ErrorInvalidParam` in URSim because FT emulation may be incomplete.
- Workaround: Skip the test by leaving `ENABLE_FT_TESTS` unset or set to `false`.

### First connection sometimes fails but subsequent tests pass
- Intermittent first-connect hiccups can occur on some hosts.
- Mitigation: Add a short retry on first failure; subsequent reconnects are stable.
