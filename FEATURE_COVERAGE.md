# UR.RTDE Feature Coverage Report

**Date**: 2025-10-27  
**C++ Library**: ur_rtde v1.6.0  
**Wrapper Version**: UR.RTDE v1.0.0  

---

## Summary

| Component | Total Methods | Implemented | Coverage |
|-----------|--------------|-------------|----------|
| **RTDEControlInterface** | ~70 | 13 | ~19% |
| **RTDEReceiveInterface** | ~50 | 10 | ~20% |
| **RTDEIOInterface** | ~10 | 2 | ~20% |
| **DashboardClient** | ~40 | 0 | 0% |
| **ScriptClient** | ~5 | 0 | 0% |
| **RobotiqGripper** | ~10 | 0 | 0% |

**Overall Coverage**: ~18% of total C++ library features

---

## ✅ Implemented Features

### RTDEControl (13/~70 methods)

#### Movement Commands
- ✅ `MoveJ(q, speed, acceleration, async)` - Joint space movement
- ✅ `MoveL(pose, speed, acceleration, async)` - Linear Cartesian movement
- ✅ `StopJ(acceleration, async)` - Stop joint movement
- ✅ `StopL(acceleration, async)` - Stop linear movement
- ✅ `SpeedJ(qd, acceleration, time)` - Joint space velocity control
- ✅ `SpeedL(xd, acceleration, time)` - Cartesian velocity control
- ✅ `ServoJ(q, speed, acceleration, time, ...)` - Real-time joint servo

#### Configuration
- ✅ `SetTcp(tcpPose)` - Set tool center point offset
- ✅ `SetPayload(mass, centerOfGravity)` - Set payload parameters

#### Connection Management
- ✅ `Disconnect()` - Disconnect from robot
- ✅ `Reconnect()` - Reconnect to robot
- ✅ `IsConnected` - Check connection status

#### Watchdog
- ✅ `KickWatchdog()` - Reset watchdog timer

### RTDEReceive (10/~50 methods)

#### Joint Data
- ✅ `GetActualQ()` - Actual joint positions (6 values)
- ✅ `GetActualQd()` - Actual joint velocities (6 values)

#### TCP Data
- ✅ `GetActualTcpPose()` - Actual TCP pose (x, y, z, rx, ry, rz)
- ✅ `GetActualTcpSpeed()` - Actual TCP velocity (6 values)

#### Robot State
- ✅ `GetRobotMode()` - Robot operational mode
- ✅ `GetSafetyMode()` - Safety system mode
- ✅ `GetRuntimeState()` - Runtime execution state

#### Digital I/O
- ✅ `GetDigitalInState(index)` - Read digital input (0-7)
- ✅ `GetDigitalOutState(index)` - Read digital output (0-7)

#### Connection
- ✅ `IsConnected` - Check connection status

---

## ❌ Not Implemented (High Priority)

### RTDEControl - Critical Missing Features

#### Advanced Movement
- ❌ `MoveJ_IK(pose, ...)` - Joint movement with IK
- ❌ `MoveL_FK(q, ...)` - Linear movement from joint config
- ❌ `MovePath(path, async)` - Path execution
- ❌ `ServoC(pose, ...)` - Circular servo movement
- ❌ `ServoL(pose, ...)` - Linear servo movement
- ❌ `ServoStop(a)` - Stop servo movement
- ❌ `SpeedStop(a)` - Stop speed movement

#### Force Control
- ❌ `ForceMode(task_frame, selection, wrench, ...)` - Enter force mode
- ❌ `ForceModeSetDamping(damping)` - Adjust force damping
- ❌ `ForceModeSetGainScaling(scaling)` - Adjust force gain
- ❌ `ForceModeStop()` - Exit force mode
- ❌ `MoveUntilContact(xd, ...)` - Move until contact detected
- ❌ `ToolContact(direction)` - Detect tool contact

#### Freedrive & Teach
- ❌ `FreedriveModeEnable(free_axes, ...)` - Enable freedrive
- ❌ `EndFreedrive()` - Disable freedrive
- ❌ `TeachMode()` - Enter teach mode
- ❌ `EndTeachMode()` - Exit teach mode

#### Jogging
- ❌ `JogStart(speeds, feature, acc, time)` - Start jogging
- ❌ `JogStop()` - Stop jogging

#### Kinematics
- ❌ `GetInverseKinematics(x, qnear, ...)` - Calculate IK
- ❌ `GetInverseKinematicsHasSolution(x, ...)` - Check IK solvability
- ❌ `GetForwardKinematics(q, ...)` - Calculate FK
- ❌ `PoseTrans(p_from, p_from_to)` - Transform poses

#### Contact Detection
- ❌ `StartContactDetection(direction)` - Enable contact detection
- ❌ `StopContactDetection()` - Disable contact detection
- ❌ `ReadContactDetection()` - Read contact state

#### Force/Torque
- ❌ `EnableExternalFtSensor(enable, mass, ...)` - External F/T sensor
- ❌ `SetExternalForceTorque(force_torque)` - Set external wrench
- ❌ `ZeroFtSensor()` - Zero force/torque sensor

#### Advanced Configuration
- ❌ `SetWatchdog(min_frequency)` - Configure watchdog
- ❌ `SetGravity(direction)` - Set gravity vector
- ❌ `SetTargetPayload(mass, cog, ...)` - Advanced payload config

#### Safety & Limits
- ❌ `IsJointsWithinSafetyLimits(q)` - Check joint limits
- ❌ `IsPoseWithinSafetyLimits(pose)` - Check Cartesian limits
- ❌ `TriggerProtectiveStop()` - Trigger safety stop

#### Status Queries
- ❌ `IsProgramRunning()` - Check if program is running
- ❌ `IsSteady()` - Check if robot is steady
- ❌ `GetAsyncOperationProgress()` - Get async operation status
- ❌ `GetAsyncOperationProgressEx()` - Extended async status
- ❌ `GetRobotStatus()` - Detailed robot status
- ❌ `GetFreediveStatus()` - Freedrive mode status

#### Data Access
- ❌ `GetActualJointPositionsHistory(steps)` - Historical joint data
- ❌ `GetActualToolFlangePose()` - Tool flange pose
- ❌ `GetJointTorques()` - Joint torques
- ❌ `GetTargetWaypoint()` - Current target waypoint
- ❌ `GetTCPOffset()` - Get TCP offset
- ❌ `GetStepTime()` - Get RTDE cycle time

#### Custom Scripting
- ❌ `SendCustomScript(script)` - Send URScript
- ❌ `SendCustomScriptFile(file_path)` - Load script file
- ❌ `SendCustomScriptFunction(name, script)` - Send function
- ❌ `SetCustomScriptFile(file_path)` - Set custom script
- ❌ `StopScript()` - Stop running script
- ❌ `ReuploadScript()` - Reload control script

### RTDEReceive - Missing Data Streams

#### Extended Joint Data
- ❌ `GetActualCurrent()` - Joint motor currents
- ❌ `GetActualJointVoltage()` - Joint voltages
- ❌ `GetJointControlOutput()` - Control output signals
- ❌ `GetJointTemperatures()` - Joint temperatures
- ❌ `GetJointMode()` - Joint control modes

#### TCP Extended Data
- ❌ `GetActualTCPForce()` - TCP force readings
- ❌ `GetActualToolAccelerometer()` - Tool accelerometer

#### Target (Commanded) Values
- ❌ `GetTargetQ()` - Target joint positions
- ❌ `GetTargetQd()` - Target joint velocities
- ❌ `GetTargetQdd()` - Target joint accelerations
- ❌ `GetTargetTCPPose()` - Target TCP pose
- ❌ `GetTargetTCPSpeed()` - Target TCP velocity
- ❌ `GetTargetCurrent()` - Target motor currents
- ❌ `GetTargetMoment()` - Target joint moments

#### Force/Torque
- ❌ `GetFtRawWrench()` - Raw F/T sensor data

#### Payload
- ❌ `GetPayload()` - Current payload mass
- ❌ `GetPayloadCog()` - Payload center of gravity
- ❌ `GetPayloadInertia()` - Payload inertia

#### System Status
- ❌ `GetActualMainVoltage()` - Main power voltage
- ❌ `GetActualRobotVoltage()` - Robot control voltage
- ❌ `GetActualRobotCurrent()` - Total robot current
- ❌ `GetActualMomentum()` - Robot momentum
- ❌ `GetActualExecutionTime()` - Execution time
- ❌ `GetSpeedScaling()` - Speed scaling factor
- ❌ `GetSpeedScalingCombined()` - Combined speed scaling
- ❌ `GetTargetSpeedFraction()` - Target speed fraction
- ❌ `GetRobotStatus()` - Detailed robot status
- ❌ `GetSafetyStatusBits()` - Safety status bits

#### Safety Checks
- ❌ `IsProtectiveStopped()` - Check protective stop
- ❌ `IsEmergencyStopped()` - Check emergency stop

#### Analog I/O
- ❌ `GetStandardAnalogInput0()` - Analog input 0
- ❌ `GetStandardAnalogInput1()` - Analog input 1
- ❌ `GetStandardAnalogOutput0()` - Analog output 0
- ❌ `GetStandardAnalogOutput1()` - Analog output 1

#### Digital I/O Extended
- ❌ `GetActualDigitalInputBits()` - All digital inputs (bitmask)
- ❌ `GetActualDigitalOutputBits()` - All digital outputs (bitmask)

#### Registers
- ❌ `GetOutputIntRegister(id)` - Integer register
- ❌ `GetOutputDoubleRegister(id)` - Double register

#### Data Recording
- ❌ `StartFileRecording(filename, variables)` - Start recording
- ❌ `StopFileRecording()` - Stop recording

### RTDEIOInterface - Missing I/O Control

- ❌ `SetStandardDigitalOut(id, level)` - Set digital output
- ❌ `SetConfigurableDigitalOut(id, level)` - Set configurable output
- ❌ `SetToolDigitalOut(id, level)` - Set tool output
- ❌ `SetAnalogOutputVoltage(id, voltage)` - Set analog voltage
- ❌ `SetAnalogOutputCurrent(id, current)` - Set analog current
- ❌ `SetInputIntRegister(id, value)` - Write integer register
- ❌ `SetInputDoubleRegister(id, value)` - Write double register
- ❌ `SetSpeedSlider(speed)` - Control speed slider

---

## ❌ Not Implemented (Additional Components)

### DashboardClient (0/~40)
Complete robot control and monitoring via dashboard server:
- Power on/off
- Brake release/engage
- Program loading and execution
- Safety status queries
- Popup messages
- Language settings
- Variable access
- Installation management

### ScriptClient (0/~5)
Direct URScript execution:
- Send URScript programs
- Real-time script injection

### RobotiqGripper (0/~10)
Robotiq gripper control:
- Activate/deactivate
- Open/close
- Position control
- Force control
- Speed control

---

## Recommendations

### Phase 1: Essential Missing Features (High Priority)

1. **Kinematics** (critical for path planning)
   - `GetInverseKinematics()`
   - `GetForwardKinematics()`
   - `GetInverseKinematicsHasSolution()`

2. **Path Execution** (for complex movements)
   - `MovePath()`
   - Path building utilities

3. **I/O Control** (RTDEIOInterface)
   - `SetStandardDigitalOut()`
   - `SetToolDigitalOut()`
   - `SetAnalogOutputVoltage()`
   - `SetAnalogOutputCurrent()`

4. **Safety Status** (RTDEReceive)
   - `IsProtectiveStopped()`
   - `IsEmergencyStopped()`
   - `GetSafetyStatusBits()`

5. **Advanced Data** (RTDEReceive)
   - `GetTargetQ()` / `GetTargetTCPPose()`
   - `GetActualTCPForce()`
   - `GetJointTemperatures()`

### Phase 2: Advanced Features (Medium Priority)

1. **Force Control**
   - `ForceMode()` and related methods
   - `MoveUntilContact()`
   - F/T sensor integration

2. **Servo Control**
   - `ServoC()` / `ServoL()`
   - `ServoStop()`

3. **Teach/Freedrive**
   - `FreedriveModeEnable()` / `EndFreedrive()`
   - `TeachMode()` / `EndTeachMode()`

4. **Custom Scripting**
   - `SendCustomScript()`
   - `SendCustomScriptFile()`

### Phase 3: Extended Features (Lower Priority)

1. **DashboardClient** - Full robot management
2. **RobotiqGripper** - Gripper integration
3. **ScriptClient** - Advanced scripting
4. **Data Recording** - Log robot data
5. **Contact Detection** - Advanced sensing

---

## Current Status: Production Ready for Basic Use

The current implementation covers **~18% of total features** but includes:

✅ **Core Movement**: MoveJ, MoveL, Speed, Servo  
✅ **Essential Data**: Joint positions, TCP pose, velocities  
✅ **Safety**: Stop commands, modes, basic I/O  
✅ **Configuration**: TCP, payload, watchdog  
✅ **Connection**: Robust lifecycle management  

This is **sufficient for**:
- Basic robot programming
- Simple pick-and-place operations
- Data monitoring and visualization
- Educational purposes
- Proof-of-concept applications

This is **NOT sufficient for**:
- Force-controlled operations
- Advanced path planning (without IK)
- Complex I/O interaction
- Gripper control
- Full robot management
- Production-grade industrial applications requiring all features

---

## Conclusion

**Current Coverage**: ~18% (core features)  
**Recommendation**: Implement Phase 1 features to reach ~40% coverage for production readiness in industrial applications.

The wrapper is **production-ready for basic robot control** but would benefit from Phase 1 additions for broader industrial use cases.
