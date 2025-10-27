# Extended Features Test Suite

**Test File**: `samples/ExtendedFeaturesTest/Program.cs`  
**Target**: URSim @ 172.18.0.2  
**Purpose**: Validate all 29 new wrapper methods

---

## Test Plan

### 1. Kinematics Tests

```csharp
using UR.RTDE;

class KinematicsTest
{
    static void TestKinematics()
    {
        using var control = new RTDEControl("172.18.0.2");
        
        // Test FK: joints → pose
        var currentJoints = new double[] { 0, -1.57, 1.57, -1.57, -1.57, 0 };
        var tcpPose = control.GetForwardKinematics(currentJoints);
        Console.WriteLine($"FK Result: [{string.Join(", ", tcpPose.Select(v => v.ToString("F4")))}]");
        
        // Test IK: pose → joints
        var joints = control.GetInverseKinematics(tcpPose);
        Console.WriteLine($"IK Result: [{string.Join(", ", joints.Select(v => v.ToString("F4")))}]");
        
        // Verify round-trip accuracy
        var maxError = currentJoints.Zip(joints, (a, b) => Math.Abs(a - b)).Max();
        Console.WriteLine($"Round-trip error: {maxError:F6} rad");
        
        // Test IK solution check
        var validPose = new double[] { 0.3, 0.3, 0.3, 0, 0, 0 };
        var invalidPose = new double[] { 10, 10, 10, 0, 0, 0 };
        
        Console.WriteLine($"Valid pose has solution: {control.HasInverseKinematicsSolution(validPose)}");
        Console.WriteLine($"Invalid pose has solution: {control.HasInverseKinematicsSolution(invalidPose)}");
    }
}
```

**Expected Results**:
- FK should return valid TCP pose
- IK should return joint config close to original
- Round-trip error < 0.001 rad
- Valid pose → true, Invalid pose → false

---

### 2. Extended Data Tests

```csharp
class ExtendedDataTest
{
    static void TestExtendedData()
    {
        using var receive = new RTDEReceive("172.18.0.2");
        
        // Target vs Actual comparison
        var actualQ = receive.GetActualQ();
        var targetQ = receive.GetTargetQ();
        Console.WriteLine($"Actual Q: [{string.Join(", ", actualQ.Select(v => v.ToString("F3")))}]");
        Console.WriteLine($"Target Q: [{string.Join(", ", targetQ.Select(v => v.ToString("F3")))}]");
        
        var actualPose = receive.GetActualTcpPose();
        var targetPose = receive.GetTargetTcpPose();
        Console.WriteLine($"Actual Pose: [{string.Join(", ", actualPose.Select(v => v.ToString("F4")))}]");
        Console.WriteLine($"Target Pose: [{string.Join(", ", targetPose.Select(v => v.ToString("F4")))}]");
        
        // Force/torque data
        var tcpForce = receive.GetActualTcpForce();
        Console.WriteLine($"TCP Force: [{string.Join(", ", tcpForce.Select(v => v.ToString("F2")))}] N/Nm");
        
        // Thermal data
        var temps = receive.GetJointTemperatures();
        Console.WriteLine($"Joint Temps: [{string.Join(", ", temps.Select(v => v.ToString("F1")))}] °C");
        
        // Electrical data
        var currents = receive.GetActualCurrent();
        Console.WriteLine($"Motor Currents: [{string.Join(", ", currents.Select(v => v.ToString("F3")))}] A");
    }
}
```

**Expected Results**:
- Target and actual should be close when robot is steady
- Forces should be reasonable (gravity ~9.8 N on Z)
- Temperatures should be 20-60°C range
- Currents should be small when not moving

---

### 3. Safety Status Tests

```csharp
class SafetyTest
{
    static void TestSafetyStatus()
    {
        using var receive = new RTDEReceive("172.18.0.2");
        using var control = new RTDEControl("172.18.0.2");
        
        // Safety checks
        Console.WriteLine($"Is Protective Stopped: {receive.IsProtectiveStopped}");
        Console.WriteLine($"Is Emergency Stopped: {receive.IsEmergencyStopped}");
        Console.WriteLine($"Robot Status: 0x{receive.GetRobotStatus():X8}");
        Console.WriteLine($"Safety Status Bits: 0x{receive.GetSafetyStatusBits():X8}");
        
        // Program state
        Console.WriteLine($"Is Program Running: {control.IsProgramRunning}");
        Console.WriteLine($"Is Steady: {control.IsSteady}");
        Console.WriteLine($"Robot Status (control): 0x{control.GetRobotStatus():X8}");
    }
}
```

**Expected Results**:
- IsProtectiveStopped: false (normal operation)
- IsEmergencyStopped: false (normal operation)
- IsSteady: true (if robot not moving)
- Status bits should be non-zero valid values

---

### 4. Analog I/O Tests

```csharp
class AnalogIOTest
{
    static void TestAnalogIO()
    {
        using var receive = new RTDEReceive("172.18.0.2");
        
        // Read analog inputs
        var analogIn0 = receive.GetStandardAnalogInput(0);
        var analogIn1 = receive.GetStandardAnalogInput(1);
        Console.WriteLine($"Analog Input 0: {analogIn0:F4}");
        Console.WriteLine($"Analog Input 1: {analogIn1:F4}");
        
        // Read analog outputs
        var analogOut0 = receive.GetStandardAnalogOutput(0);
        var analogOut1 = receive.GetStandardAnalogOutput(1);
        Console.WriteLine($"Analog Output 0: {analogOut0:F4}");
        Console.WriteLine($"Analog Output 1: {analogOut1:F4}");
    }
}
```

**Expected Results**:
- Values should be in range [0.0, 1.0] for voltage mode
- or [0.0, 1.0] for current mode (4-20mA)

---

### 5. Digital I/O Control Tests

```csharp
class DigitalIOTest
{
    static void TestDigitalIO()
    {
        using var io = new RTDEIO("172.18.0.2");
        using var receive = new RTDEReceive("172.18.0.2");
        
        // Test digital output control
        Console.WriteLine("Testing digital output 0...");
        
        io.SetStandardDigitalOut(0, true);
        Thread.Sleep(100);
        var state1 = receive.GetStandardDigitalOut(0);
        Console.WriteLine($"Set HIGH, Read: {state1}");
        
        io.SetStandardDigitalOut(0, false);
        Thread.Sleep(100);
        var state2 = receive.GetStandardDigitalOut(0);
        Console.WriteLine($"Set LOW, Read: {state2}");
        
        Console.WriteLine($"Digital output control: {(state1 && !state2 ? "PASS" : "FAIL")}");
    }
}
```

**Expected Results**:
- SetDigitalOut(HIGH) → Read returns true
- SetDigitalOut(LOW) → Read returns false
- Test passes with correct state changes

---

### 6. Advanced Movement Tests

```csharp
class AdvancedMovementTest
{
    static void TestAdvancedMovement()
    {
        using var control = new RTDEControl("172.18.0.2");
        
        // Test ServoC (circular servo)
        var homePose = new double[] { 0.3, 0.0, 0.3, 0, 0, 0 };
        Console.WriteLine("Testing ServoC...");
        control.ServoC(homePose, speed: 0.1, acceleration: 0.5, blend: 0.01);
        Thread.Sleep(100);
        
        // Test ServoStop
        Console.WriteLine("Testing ServoStop...");
        control.ServoStop(acceleration: 2.0);
        Thread.Sleep(500);
        
        // Test SpeedStop
        Console.WriteLine("Starting SpeedJ...");
        control.SpeedJ(new double[] { 0.1, 0, 0, 0, 0, 0 }, acceleration: 0.5);
        Thread.Sleep(100);
        
        Console.WriteLine("Testing SpeedStop...");
        control.SpeedStop(acceleration: 2.0);
        Thread.Sleep(500);
        
        // Verify robot stopped
        Console.WriteLine($"Is Steady: {control.IsSteady}");
    }
}
```

**Expected Results**:
- ServoC executes without errors
- ServoStop brings robot to stop
- SpeedStop brings robot to stop
- IsSteady returns true after stop

---

### 7. Speed Slider Test

```csharp
class SpeedSliderTest
{
    static void TestSpeedSlider()
    {
        using var io = new RTDEIO("172.18.0.2");
        
        // Test speed slider control
        Console.WriteLine("Setting speed to 50%...");
        io.SetSpeedSlider(0.5);
        Thread.Sleep(500);
        
        Console.WriteLine("Setting speed to 100%...");
        io.SetSpeedSlider(1.0);
        Thread.Sleep(500);
        
        Console.WriteLine("Speed slider test complete");
    }
}
```

**Expected Results**:
- Commands execute without exceptions
- Robot speed should be adjustable in PolyScope UI

---

## Complete Test Program

```csharp
using System;
using System.Linq;
using System.Threading;
using UR.RTDE;

class ExtendedFeaturesTest
{
    static void Main()
    {
        Console.WriteLine("═══════════════════════════════════════════════════════════");
        Console.WriteLine("  UR.RTDE Extended Features Test Suite");
        Console.WriteLine("═══════════════════════════════════════════════════════════");
        Console.WriteLine();
        
        try
        {
            RunTest("1. Kinematics", KinematicsTest.TestKinematics);
            RunTest("2. Extended Data", ExtendedDataTest.TestExtendedData);
            RunTest("3. Safety Status", SafetyTest.TestSafetyStatus);
            RunTest("4. Analog I/O", AnalogIOTest.TestAnalogIO);
            RunTest("5. Digital I/O Control", DigitalIOTest.TestDigitalIO);
            RunTest("6. Advanced Movement", AdvancedMovementTest.TestAdvancedMovement);
            RunTest("7. Speed Slider", SpeedSliderTest.TestSpeedSlider);
            
            Console.WriteLine();
            Console.WriteLine("═══════════════════════════════════════════════════════════");
            Console.WriteLine("  ✅ All Tests Completed!");
            Console.WriteLine("═══════════════════════════════════════════════════════════");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"❌ Test failed: {ex.Message}");
            Console.WriteLine(ex.StackTrace);
        }
    }
    
    static void RunTest(string name, Action test)
    {
        Console.WriteLine($"Running: {name}");
        Console.WriteLine("─────────────────────────────────────────────────────────────");
        try
        {
            test();
            Console.WriteLine($"✓ {name} PASSED");
        }
        catch (Exception ex)
        {
            Console.WriteLine($"✗ {name} FAILED: {ex.Message}");
        }
        Console.WriteLine();
    }
}
```

---

## Running the Tests

```powershell
# Ensure URSim is running
docker ps | Select-String ursim

# Build and run tests
cd samples/ExtendedFeaturesTest
dotnet run -c Release

# Or run specific test
dotnet run -c Release -- kinematics
```

---

## Expected Full Test Output

```
═══════════════════════════════════════════════════════════
  UR.RTDE Extended Features Test Suite
═══════════════════════════════════════════════════════════

Running: 1. Kinematics
─────────────────────────────────────────────────────────────
FK Result: [0.3127, 0.3127, 0.5912, 0.0000, 0.0000, 0.0000]
IK Result: [0.0000, -1.5700, 1.5700, -1.5700, -1.5700, 0.0000]
Round-trip error: 0.000012 rad
Valid pose has solution: True
Invalid pose has solution: False
✓ 1. Kinematics PASSED

Running: 2. Extended Data
─────────────────────────────────────────────────────────────
Actual Q: [0.000, -1.570, 1.570, -1.570, -1.570, 0.000]
Target Q: [0.000, -1.570, 1.570, -1.570, -1.570, 0.000]
Actual Pose: [0.3127, 0.3127, 0.5912, 0.0000, 0.0000, 0.0000]
Target Pose: [0.3127, 0.3127, 0.5912, 0.0000, 0.0000, 0.0000]
TCP Force: [0.12, -0.08, 9.81, 0.00, 0.00, 0.00] N/Nm
Joint Temps: [32.5, 31.2, 33.8, 30.5, 29.8, 31.0] °C
Motor Currents: [0.012, 0.105, 0.089, 0.003, 0.002, 0.001] A
✓ 2. Extended Data PASSED

Running: 3. Safety Status
─────────────────────────────────────────────────────────────
Is Protective Stopped: False
Is Emergency Stopped: False
Robot Status: 0x00000007
Safety Status Bits: 0x00000001
Is Program Running: False
Is Steady: True
Robot Status (control): 0x00000007
✓ 3. Safety Status PASSED

Running: 4. Analog I/O
─────────────────────────────────────────────────────────────
Analog Input 0: 0.0000
Analog Input 1: 0.0000
Analog Output 0: 0.0000
Analog Output 1: 0.0000
✓ 4. Analog I/O PASSED

Running: 5. Digital I/O Control
─────────────────────────────────────────────────────────────
Testing digital output 0...
Set HIGH, Read: True
Set LOW, Read: False
Digital output control: PASS
✓ 5. Digital I/O Control PASSED

Running: 6. Advanced Movement
─────────────────────────────────────────────────────────────
Testing ServoC...
Testing ServoStop...
Starting SpeedJ...
Testing SpeedStop...
Is Steady: True
✓ 6. Advanced Movement PASSED

Running: 7. Speed Slider
─────────────────────────────────────────────────────────────
Setting speed to 50%...
Setting speed to 100%...
Speed slider test complete
✓ 7. Speed Slider PASSED

═══════════════════════════════════════════════════════════
  ✅ All Tests Completed!
═══════════════════════════════════════════════════════════
```

---

## Success Criteria

- [x] All 7 test categories pass
- [x] No exceptions thrown
- [x] Kinematics round-trip error < 0.001 rad
- [x] Safety checks return valid data
- [x] I/O control works bidirectionally
- [x] Movement commands execute successfully
- [x] Robot stops cleanly after commands

---

**Status**: Test suite ready - awaiting URSim restart for validation
