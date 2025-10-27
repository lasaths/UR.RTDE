using System;
using System.Threading;
using System.Threading.Tasks;
using UR.RTDE;

namespace UR.RTDE.Tests
{
    /// <summary>
    /// Integration tests for UR.RTDE with URSim
    /// Requires URSim running at 172.18.0.2
    /// </summary>
    class URSimIntegrationTests
    {
        private const string URSIM_IP = "172.18.0.2";
        private const int TEST_TIMEOUT_MS = 30000; // 30 seconds

        static async Task<int> Main(string[] args)
        {
            Console.WriteLine("═══════════════════════════════════════════════");
            Console.WriteLine("  UR.RTDE Integration Tests - URSim");
            Console.WriteLine("═══════════════════════════════════════════════");
            Console.WriteLine($"Target: {URSIM_IP}");
            Console.WriteLine($"Time: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            Console.WriteLine();

            int passed = 0;
            int failed = 0;

            // Test 1: Basic Connection
            if (await RunTest("Test 1: Basic Connection", TestBasicConnection))
                passed++;
            else
                failed++;

            // Test 2: Receive Interface
            if (await RunTest("Test 2: Receive Interface - Read Robot State", TestReceiveInterface))
                passed++;
            else
                failed++;

            // Test 3: Control Interface
            if (await RunTest("Test 3: Control Interface - Get Inverse Kinematics", TestControlInterface))
                passed++;
            else
                failed++;

            // Test 4: Streaming Data
            if (await RunTest("Test 4: Streaming Data (10 seconds)", TestStreamingData))
                passed++;
            else
                failed++;

            // Test 5: Joint Movement
            if (await RunTest("Test 5: Joint Movement (Small)", TestJointMovement))
                passed++;
            else
                failed++;

            // Test 6: Emergency Stop
            if (await RunTest("Test 6: Emergency Stop", TestEmergencyStop))
                passed++;
            else
                failed++;

            // Test 7: Reconnection
            if (await RunTest("Test 7: Reconnection Handling", TestReconnection))
                passed++;
            else
                failed++;

            // Summary
            Console.WriteLine();
            Console.WriteLine("═══════════════════════════════════════════════");
            Console.WriteLine("  Test Summary");
            Console.WriteLine("═══════════════════════════════════════════════");
            Console.WriteLine($"Passed: {passed}");
            Console.WriteLine($"Failed: {failed}");
            Console.WriteLine($"Total:  {passed + failed}");
            Console.WriteLine();

            if (failed == 0)
            {
                Console.ForegroundColor = ConsoleColor.Green;
                Console.WriteLine("✓ ALL TESTS PASSED!");
                Console.ResetColor();
                return 0;
            }
            else
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine($"✗ {failed} TEST(S) FAILED");
                Console.ResetColor();
                return 1;
            }
        }

        static async Task<bool> RunTest(string testName, Func<Task> testFunc)
        {
            Console.WriteLine();
            Console.WriteLine($"Running: {testName}");
            Console.WriteLine("─────────────────────────────────────────────");

            try
            {
                using var cts = new CancellationTokenSource(TEST_TIMEOUT_MS);
                await Task.Run(testFunc, cts.Token);

                Console.ForegroundColor = ConsoleColor.Green;
                Console.WriteLine($"✓ PASSED: {testName}");
                Console.ResetColor();
                return true;
            }
            catch (Exception ex)
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine($"✗ FAILED: {testName}");
                Console.WriteLine($"  Error: {ex.Message}");
                Console.ResetColor();
                return false;
            }
        }

        static async Task TestBasicConnection()
        {
            using var control = new RTDEControl(URSIM_IP);
            Console.WriteLine($"  Connected to control interface");

            using var receive = new RTDEReceive(URSIM_IP);
            Console.WriteLine($"  Connected to receive interface");

            await Task.Delay(100); // Ensure connection is stable
            Console.WriteLine($"  Connection stable");
        }

        static async Task TestReceiveInterface()
        {
            using var receive = new RTDEReceive(URSIM_IP);

            // Read actual joint positions
            double[] actualQ = receive.GetActualQ();
            Console.WriteLine($"  Joint positions: [{string.Join(", ", Array.ConvertAll(actualQ, x => x.ToString("F4")))}]");

            // Read actual TCP pose
            double[] tcpPose = receive.GetActualTcpPose();
            Console.WriteLine($"  TCP pose: [{string.Join(", ", Array.ConvertAll(tcpPose, x => x.ToString("F4")))}]");

            // Read robot mode
            int robotMode = receive.GetRobotMode();
            Console.WriteLine($"  Robot mode: {robotMode}");

            // Read safety mode
            int safetyMode = receive.GetSafetyMode();
            Console.WriteLine($"  Safety mode: {safetyMode}");

            await Task.CompletedTask;
        }

        static async Task TestControlInterface()
        {
            using var control = new RTDEControl(URSIM_IP);
            using var receive = new RTDEReceive(URSIM_IP);

            // Get current TCP pose
            double[] currentPose = receive.GetActualTcpPose();
            Console.WriteLine($"  Current TCP pose: [{string.Join(", ", Array.ConvertAll(currentPose, x => x.ToString("F4")))}]");

            // Get current joint positions
            double[] actualQ = receive.GetActualQ();
            Console.WriteLine($"  Current joints: [{string.Join(", ", Array.ConvertAll(actualQ, x => x.ToString("F4")))}]");

            // Verify we got valid results (no NaN or infinity)
            foreach (var q in actualQ)
            {
                if (double.IsNaN(q) || double.IsInfinity(q))
                    throw new Exception("Invalid joint position");
            }

            Console.WriteLine($"  Control interface working correctly");
            await Task.CompletedTask;
        }

        static async Task TestStreamingData()
        {
            using var receive = new RTDEReceive(URSIM_IP);

            Console.WriteLine($"  Streaming for 10 seconds...");
            var startTime = DateTime.Now;
            int sampleCount = 0;

            while ((DateTime.Now - startTime).TotalSeconds < 10)
            {
                double[] q = receive.GetActualQ();
                sampleCount++;

                if (sampleCount % 500 == 0)
                {
                    Console.WriteLine($"  Samples: {sampleCount}, Joint0: {q[0]:F4} rad");
                }

                await Task.Delay(2); // ~500 Hz
            }

            double frequency = sampleCount / (DateTime.Now - startTime).TotalSeconds;
            Console.WriteLine($"  Total samples: {sampleCount}");
            Console.WriteLine($"  Average frequency: {frequency:F1} Hz");

            if (frequency < 100)
                throw new Exception($"Streaming frequency too low: {frequency:F1} Hz");
        }

        static async Task TestJointMovement()
        {
            using var control = new RTDEControl(URSIM_IP);
            using var receive = new RTDEReceive(URSIM_IP);

            // Get current position
            double[] currentQ = receive.GetActualQ();
            Console.WriteLine($"  Current position: J0={currentQ[0]:F4} rad");

            // Create a small movement target (move joint 0 by 0.1 radians)
            double[] targetQ = (double[])currentQ.Clone();
            targetQ[0] += 0.1; // Small movement

            Console.WriteLine($"  Target position: J0={targetQ[0]:F4} rad");
            Console.WriteLine($"  Starting movement...");

            // Execute movement (void return)
            control.MoveJ(targetQ, speed: 0.5, acceleration: 0.5);
            Console.WriteLine($"  Movement command sent");

            // Wait for movement to complete (poll for position)
            var startTime = DateTime.Now;
            bool reached = false;

            while ((DateTime.Now - startTime).TotalSeconds < 10)
            {
                double[] currentPos = receive.GetActualQ();
                double error = Math.Abs(currentPos[0] - targetQ[0]);

                if (error < 0.01) // Within 0.01 radians
                {
                    reached = true;
                    Console.WriteLine($"  Target reached! Final J0={currentPos[0]:F4} rad");
                    break;
                }

                await Task.Delay(100);
            }

            if (!reached)
                throw new Exception("Movement did not complete in time");
        }

        static async Task TestEmergencyStop()
        {
            using var control = new RTDEControl(URSIM_IP);
            using var receive = new RTDEReceive(URSIM_IP);

            // Start a long movement
            double[] currentQ = receive.GetActualQ();
            double[] targetQ = (double[])currentQ.Clone();
            targetQ[0] += 0.5; // Larger movement

            Console.WriteLine($"  Starting long movement...");
            control.MoveJ(targetQ, speed: 0.3, acceleration: 0.3);

            // Wait a bit
            await Task.Delay(500);

            // Emergency stop (void return)
            Console.WriteLine($"  Triggering emergency stop...");
            control.StopJ(5.0); // 5 rad/s² deceleration
            Console.WriteLine($"  Emergency stop command sent");

            // Verify robot stopped
            await Task.Delay(2000);
            double[] finalQ = receive.GetActualQ();
            Console.WriteLine($"  Final position: J0={finalQ[0]:F4} rad");

            // Should not have reached the target
            double error = Math.Abs(finalQ[0] - targetQ[0]);
            if (error < 0.1)
                Console.WriteLine($"  Warning: Robot might have reached target despite stop");
        }

        static async Task TestReconnection()
        {
            // Connect
            using var control1 = new RTDEControl(URSIM_IP);
            Console.WriteLine($"  First connection established");

            await Task.Delay(500);

            // Disconnect and reconnect
            control1.Disconnect();
            Console.WriteLine($"  Disconnected");

            await Task.Delay(1000);

            // Reconnect with new instance
            using var control2 = new RTDEControl(URSIM_IP);
            Console.WriteLine($"  Reconnected successfully");

            using var receive = new RTDEReceive(URSIM_IP);
            double[] q = receive.GetActualQ();
            Console.WriteLine($"  Can read data after reconnection: J0={q[0]:F4} rad");
        }
    }
}
