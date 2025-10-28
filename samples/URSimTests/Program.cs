using System;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using UR.RTDE;

namespace URSimTests
{
    class Program
    {
        private const string ROBOT_IP = "localhost"; // URSim e-Series 5.23.0 @ localhost
        private static int _testsPassed = 0;
        private static int _testsFailed = 0;

        static async Task<int> Main(string[] args)
        {
            Console.WriteLine("=".PadRight(80, '='));
            Console.WriteLine("UR.RTDE - URSim Integration Tests");
            Console.WriteLine("Target: URSim e-Series 5.23.0 @ " + ROBOT_IP);
            Console.WriteLine("=".PadRight(80, '='));
            Console.WriteLine();

            try
            {
                // Basic connectivity tests
                await RunTest("Test 1: RTDEControl Connection", TestControlConnection);
                await RunTest("Test 2: RTDEReceive Connection", TestReceiveConnection);
                await RunTest("Test 3: RTDEIO Connection", TestIOConnection);

                // Data reading tests
                await RunTest("Test 4: Read Joint Positions", TestReadJointPositions);
                await RunTest("Test 5: Read TCP Pose", TestReadTcpPose);
                await RunTest("Test 6: Read Robot Status", TestReadRobotStatus);
                await RunTest("Test 7: Read Safety Status", TestReadSafetyStatus);
                await RunTest("Test 8: Read Digital I/O", TestReadDigitalIO);
                await RunTest("Test 9: Read Analog I/O", TestReadAnalogIO);

                // Control tests
                await RunTest("Test 10: MoveJ Execution", TestMoveJ);
                await RunTest("Test 11: MoveL Execution", TestMoveL);
                await RunTest("Test 12: SpeedJ Control", TestSpeedJ);
                await RunTest("Test 13: SpeedL Control", TestSpeedL);
                await RunTest("Test 14: ServoJ Control", TestServoJ);
                await RunTest("Test 15: Stop Commands", TestStopCommands);

                // Kinematics tests
                await RunTest("Test 16: Forward Kinematics", TestForwardKinematics);
                await RunTest("Test 17: Inverse Kinematics", TestInverseKinematics);

                // Advanced tests
                await RunTest("Test 18: Set TCP", TestSetTcp);
                await RunTest("Test 19: Set Payload", TestSetPayload);
                await RunTest("Test 20: Watchdog", TestWatchdog);

                // Streaming test
                await RunTest("Test 21: Continuous Streaming (30s)", TestContinuousStreaming);

                // Digital I/O test (if available)
                await RunTest("Test 22: Set Digital Output", TestSetDigitalOutput);

                // Advanced feature tests
                Console.WriteLine("\n--- Advanced Features ---");
                await AdvancedTests.RunAllTests();

                Console.WriteLine();
                Console.WriteLine("=".PadRight(80, '='));
                Console.WriteLine($"Test Summary: {_testsPassed} passed, {_testsFailed} failed");
                Console.WriteLine("=".PadRight(80, '='));

                return _testsFailed > 0 ? 1 : 0;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"\n[FATAL ERROR] {ex.Message}");
                Console.WriteLine(ex.StackTrace);
                return 1;
            }
        }

        static async Task RunTest(string name, Func<Task> test)
        {
            Console.Write($"{name}... ");
            try
            {
                await test();
                Console.WriteLine("[PASS]");
                _testsPassed++;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[FAIL] {ex.Message}");
                _testsFailed++;
            }
        }

        // ====================================================================
        // Connection Tests
        // ====================================================================

        static async Task TestControlConnection()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            if (!ctrl.IsConnected)
                throw new Exception("Failed to connect");
            await Task.CompletedTask;
        }

        static async Task TestReceiveConnection()
        {
            using var recv = new RTDEReceive(ROBOT_IP);
            if (!recv.IsConnected)
                throw new Exception("Failed to connect");
            await Task.CompletedTask;
        }

        static async Task TestIOConnection()
        {
            using var io = new RTDEIO(ROBOT_IP);
            if (!io.IsConnected)
                throw new Exception("Failed to connect");
            await Task.CompletedTask;
        }

        // ====================================================================
        // Data Reading Tests
        // ====================================================================

        static async Task TestReadJointPositions()
        {
            using var recv = new RTDEReceive(ROBOT_IP);
            var q = recv.GetActualQ();
            
            if (q.Length != 6)
                throw new Exception($"Expected 6 joints, got {q.Length}");
            
            if (q.All(x => Math.Abs(x) < 0.001))
                throw new Exception("All joint positions are zero (suspicious)");
            
            await Task.CompletedTask;
        }

        static async Task TestReadTcpPose()
        {
            using var recv = new RTDEReceive(ROBOT_IP);
            var pose = recv.GetActualTcpPose();
            
            if (pose.Length != 6)
                throw new Exception($"Expected 6 DOF pose, got {pose.Length}");
            
            // Check that position is reasonable (within robot workspace)
            if (Math.Abs(pose[0]) > 2.0 || Math.Abs(pose[1]) > 2.0 || Math.Abs(pose[2]) > 2.0)
                throw new Exception($"TCP position out of expected range: [{pose[0]:F3}, {pose[1]:F3}, {pose[2]:F3}]");
            
            await Task.CompletedTask;
        }

        static async Task TestReadRobotStatus()
        {
            using var recv = new RTDEReceive(ROBOT_IP);
            
            var robotMode = recv.GetRobotMode();
            var safetyMode = recv.GetSafetyMode();
            var runtimeState = recv.GetRuntimeState();
            var robotStatus = recv.GetRobotStatus();
            
            // Just verify they return values (specific values depend on robot state)
            if (robotMode < 0) throw new Exception("Invalid robot mode");
            if (safetyMode < 0) throw new Exception("Invalid safety mode");
            
            await Task.CompletedTask;
        }

        static async Task TestReadSafetyStatus()
        {
            using var recv = new RTDEReceive(ROBOT_IP);
            
            var safetyBits = recv.GetSafetyStatusBits();
            var isProtectiveStopped = recv.IsProtectiveStopped;
            var isEmergencyStopped = recv.IsEmergencyStopped;
            
            // Just verify they return without error
            await Task.CompletedTask;
        }

        static async Task TestReadDigitalIO()
        {
            using var recv = new RTDEReceive(ROBOT_IP);
            
            for (int i = 0; i < 8; i++)
            {
                var inState = recv.GetStandardDigitalIn(i);
                var outState = recv.GetStandardDigitalOut(i);
                // Values don't matter, just checking they don't crash
            }
            
            await Task.CompletedTask;
        }

        static async Task TestReadAnalogIO()
        {
            using var recv = new RTDEReceive(ROBOT_IP);
            
            var ain0 = recv.GetStandardAnalogInput(0);
            var ain1 = recv.GetStandardAnalogInput(1);
            var aout0 = recv.GetStandardAnalogOutput(0);
            var aout1 = recv.GetStandardAnalogOutput(1);
            
            // Just verify they return without error
            await Task.CompletedTask;
        }

        // ====================================================================
        // Control Tests
        // ====================================================================

        static async Task TestMoveJ()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            using var recv = new RTDEReceive(ROBOT_IP);
            
            // Get current position
            var currentQ = recv.GetActualQ();
            
            // Make a small movement on joint 1
            var targetQ = (double[])currentQ.Clone();
            targetQ[0] += 0.1; // Move 0.1 rad (~5.7 degrees)
            
            // Execute movement (non-blocking)
            ctrl.MoveJ(targetQ, speed: 0.5, acceleration: 0.5, asynchronous: true);
            
            // Wait a bit for movement to start
            await Task.Delay(1000);
            
            // Verify robot is moving or has moved
            if (!ctrl.IsProgramRunning && ctrl.IsSteady)
            {
                // Movement might have completed already (it's a small move)
                var newQ = recv.GetActualQ();
                var moved = Math.Abs(newQ[0] - currentQ[0]) > 0.01;
                if (!moved)
                    throw new Exception("Robot did not move");
            }
            
            // Stop any remaining movement
            ctrl.StopJ();
            await Task.Delay(500);
        }

        static async Task TestMoveL()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            using var recv = new RTDEReceive(ROBOT_IP);
            
            // Get current TCP pose
            var currentPose = recv.GetActualTcpPose();
            
            // Make a small movement in Z direction
            var targetPose = (double[])currentPose.Clone();
            targetPose[2] += 0.05; // Move 5cm up
            
            // Execute movement (non-blocking)
            ctrl.MoveL(targetPose, speed: 0.1, acceleration: 0.5, asynchronous: true);
            
            // Wait for movement to start
            await Task.Delay(1000);
            
            // Stop movement
            ctrl.StopL();
            await Task.Delay(500);
        }

        static async Task TestSpeedJ()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            
            // Command a slow joint velocity on joint 0
            var qd = new double[6] { 0.05, 0, 0, 0, 0, 0 }; // 0.05 rad/s on joint 0
            
            ctrl.SpeedJ(qd, acceleration: 0.5, time: 1.0);
            await Task.Delay(1500);
            
            // Stop
            ctrl.SpeedStop();
            await Task.Delay(500);
        }

        static async Task TestSpeedL()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            
            // Command a slow TCP velocity in Z direction
            var xd = new double[6] { 0, 0, 0.01, 0, 0, 0 }; // 1cm/s in Z
            
            ctrl.SpeedL(xd, acceleration: 0.1, time: 1.0);
            await Task.Delay(1500);
            
            // Stop
            ctrl.SpeedStop();
            await Task.Delay(500);
        }

        static async Task TestServoJ()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            using var recv = new RTDEReceive(ROBOT_IP);
            
            // Get current position
            var q = recv.GetActualQ();
            
            // Servo to a slightly different position multiple times
            for (int i = 0; i < 10; i++)
            {
                var targetQ = (double[])q.Clone();
                targetQ[0] += 0.001 * Math.Sin(i * 0.5); // Small sinusoidal movement
                
                ctrl.ServoJ(targetQ, time: 0.002, lookaheadTime: 0.1, gain: 300);
                await Task.Delay(2); // 2ms = 500Hz
            }
            
            ctrl.ServoStop();
            await Task.Delay(500);
        }

        static async Task TestStopCommands()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            
            // Test StopJ
            ctrl.StopJ(acceleration: 2.0, asynchronous: true);
            await Task.Delay(200);
            
            // Test StopL
            ctrl.StopL(acceleration: 5.0, asynchronous: true);
            await Task.Delay(200);
            
            // Test ServoStop
            ctrl.ServoStop(acceleration: 5.0);
            await Task.Delay(200);
            
            // Test SpeedStop
            ctrl.SpeedStop(acceleration: 5.0);
            await Task.Delay(200);
        }

        // ====================================================================
        // Kinematics Tests
        // ====================================================================

        static async Task TestForwardKinematics()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            using var recv = new RTDEReceive(ROBOT_IP);
            
            // Get current joint positions
            var q = recv.GetActualQ();
            
            // Calculate forward kinematics
            var calculatedPose = ctrl.GetForwardKinematics(q);
            
            // Compare with actual TCP pose
            var actualPose = recv.GetActualTcpPose();
            
            // They should be very close (within a few mm/deg)
            for (int i = 0; i < 3; i++)
            {
                if (Math.Abs(calculatedPose[i] - actualPose[i]) > 0.01) // 10mm tolerance
                    throw new Exception($"FK position mismatch: calculated={calculatedPose[i]:F4}, actual={actualPose[i]:F4}");
            }
            
            await Task.CompletedTask;
        }

        static async Task TestInverseKinematics()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            using var recv = new RTDEReceive(ROBOT_IP);
            
            // Get current TCP pose
            var pose = recv.GetActualTcpPose();
            
            // Calculate inverse kinematics
            var calculatedQ = ctrl.GetInverseKinematics(pose);
            
            // Verify solution exists
            var hasSolution = ctrl.HasInverseKinematicsSolution(pose);
            if (!hasSolution)
                throw new Exception("IK solution should exist for current pose");
            
            // Compare with actual joint positions (might be different due to multiple solutions)
            var actualQ = recv.GetActualQ();
            
            // Forward kinematics of calculated joints should match original pose
            var verifyPose = ctrl.GetForwardKinematics(calculatedQ);
            for (int i = 0; i < 3; i++)
            {
                if (Math.Abs(verifyPose[i] - pose[i]) > 0.01)
                    throw new Exception($"IK verification failed: position mismatch");
            }
            
            await Task.CompletedTask;
        }

        // ====================================================================
        // Configuration Tests
        // ====================================================================

        static async Task TestSetTcp()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            
            // Set a small TCP offset
            var tcpOffset = new double[6] { 0, 0, 0.1, 0, 0, 0 }; // 10cm in Z
            ctrl.SetTcp(tcpOffset);
            await Task.Delay(200);
            
            // Reset to zero
            var zeroTcp = new double[6];
            ctrl.SetTcp(zeroTcp);
            await Task.Delay(200);
        }

        static async Task TestSetPayload()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            
            // Set a 2kg payload at origin
            var cog = new double[3] { 0, 0, 0 };
            ctrl.SetPayload(2.0, cog);
            await Task.Delay(200);
            
            // Reset to zero
            ctrl.SetPayload(0.0, cog);
            await Task.Delay(200);
        }

        static async Task TestWatchdog()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            
            // Trigger watchdog
            ctrl.TriggerWatchdog();
            await Task.Delay(100);
            
            // Should not throw
        }

        // ====================================================================
        // Streaming Test
        // ====================================================================

        static async Task TestContinuousStreaming()
        {
            using var recv = new RTDEReceive(ROBOT_IP);
            
            int sampleCount = 0;
            var startTime = DateTime.UtcNow;
            
            // Stream for 30 seconds without artificial delay
            while ((DateTime.UtcNow - startTime).TotalSeconds < 30)
            {
                var q = recv.GetActualQ();
                var pose = recv.GetActualTcpPose();
                sampleCount++;
            }
            
            var duration = (DateTime.UtcNow - startTime).TotalSeconds;
            var frequency = sampleCount / duration;
            
            Console.Write($" ({sampleCount} samples @ {frequency:F0} Hz) ");
            
            if (frequency < 100)
                throw new Exception($"Streaming too slow: {frequency:F0} Hz");
            
            await Task.CompletedTask;
        }

        // ====================================================================
        // Digital I/O Test
        // ====================================================================

        static async Task TestSetDigitalOutput()
        {
            using var io = new RTDEIO(ROBOT_IP);
            
            // Try to set digital output 0
            io.SetStandardDigitalOut(0, true);
            await Task.Delay(100);
            
            io.SetStandardDigitalOut(0, false);
            await Task.Delay(100);
            
            // Just verify it doesn't crash
        }
    }
}
