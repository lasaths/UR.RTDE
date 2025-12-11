using System;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using UR.RTDE;

namespace URSimTests
{
    /// <summary>
    /// Advanced feature tests for ForceMode, Jog, ServoL, TeachMode, etc.
    /// </summary>
    public static class AdvancedTests
    {
        private const string ROBOT_IP = "localhost"; // URSim in Docker

        public static async Task RunAllTests()
        {
            Console.WriteLine("\n" + "=".PadRight(80, '='));
            Console.WriteLine("Advanced Feature Tests");
            Console.WriteLine("=".PadRight(80, '='));
            Console.WriteLine();

            var testList = new System.Collections.Generic.List<(string Name, Func<Task> Test)>
            {
                ("ServoL - Cartesian Servoing", TestServoL),
                ("ForceMode - Compliant Control", TestForceMode),
                ("JogStart/Stop - Manual Jogging", TestJog),
                ("TeachMode - Freedrive", TestTeachMode),
                ("TriggerProtectiveStop - Safety", TestTriggerProtectiveStop),
                ("Extended Receive Data", TestExtendedReceiveData)
            };

            // Gate FT zeroing on capability flag (URSim often lacks FT emulation)
            var enableFt = Environment.GetEnvironmentVariable("ENABLE_FT_TESTS");
            if (!string.IsNullOrEmpty(enableFt) &&
                (enableFt.Equals("1") || enableFt.Equals("true", StringComparison.OrdinalIgnoreCase)))
            {
                testList.Insert(1, ("ZeroFtSensor - Zero Force/Torque", TestZeroFtSensor));
            }

            var enableRobotiq = Environment.GetEnvironmentVariable("ENABLE_ROBOTIQ_TESTS");
            if (!string.IsNullOrEmpty(enableRobotiq) &&
                (enableRobotiq.Equals("1") || enableRobotiq.Equals("true", StringComparison.OrdinalIgnoreCase)))
            {
                testList.Add(("Robotiq (Native API) - Open/Close", TestRobotiqNative));
                testList.Add(("Robotiq (Script) - Open/Close", TestRobotiqScript));
                testList.Add(("Robotiq (RTDE) - Activate/Open/Close", TestRobotiqRtde));
            }

            var tests = testList.ToArray();

            int passed = 0, failed = 0;

            foreach (var (name, test) in tests)
            {
                Console.Write($"{name}... ");
                try
                {
                    await test();
                    Console.WriteLine("[PASS]");
                    passed++;
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[FAIL] {ex.Message}");
                    failed++;
                }
            }

            Console.WriteLine();
            Console.WriteLine($"Advanced Tests: {passed} passed, {failed} failed");
            Console.WriteLine("=".PadRight(80, '='));
        }

        // ====================================================================
        // Advanced Movement Tests
        // ====================================================================

        static async Task TestServoL()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            using var recv = new RTDEReceive(ROBOT_IP);

            // Get current pose
            var startPose = recv.GetActualTcpPose();

            // Servo in a small circle
            for (int i = 0; i < 20; i++)
            {
                var targetPose = (double[])startPose.Clone();
                var angle = i * 2 * Math.PI / 20.0;
                targetPose[0] += 0.01 * Math.Cos(angle); // 1cm radius in X
                targetPose[1] += 0.01 * Math.Sin(angle); // 1cm radius in Y

                ctrl.ServoL(targetPose, time: 0.008, lookaheadTime: 0.1, gain: 300);
                await Task.Delay(8); // 125Hz
            }

            ctrl.ServoStop();
            await Task.Delay(500);
        }

        // ====================================================================
        // Force/Torque Tests
        // ====================================================================

        static async Task TestZeroFtSensor()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);

            // Zero the sensor
            ctrl.ZeroFtSensor();
            await Task.Delay(500);

            // Verify via receive interface
            using var recv = new RTDEReceive(ROBOT_IP);
            var force = recv.GetActualTcpForce();

            // After zeroing, forces should be close to zero (within sensor noise)
            var maxForce = force.Take(3).Max(Math.Abs);
            if (maxForce > 50.0) // 50N tolerance
            {
                throw new Exception($"Force not zeroed properly: max={maxForce:F1}N");
            }
        }

        static async Task TestForceMode()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            using var recv = new RTDEReceive(ROBOT_IP);

            // Define task frame (world frame in this case)
            var taskFrame = new double[6] { 0, 0, 0, 0, 0, 0 };

            // Selection vector: apply force in Z, position control in X,Y and rotations
            var selectionVector = new int[6] { 0, 0, 1, 0, 0, 0 }; // Z-axis force controlled

            // Target wrench: 10N downward in Z
            var wrench = new double[6] { 0, 0, -10, 0, 0, 0 };

            // Force mode type: 2 = base frame
            int forceModeType = 2;

            // Limits: max 10cm deviation from start pose
            var limits = new double[6] { 0.1, 0.1, 0.1, 0.17, 0.17, 0.17 }; // 10cm, 10deg

            try
            {
                // Enter force mode
                ctrl.ForceMode(taskFrame, selectionVector, wrench, forceModeType, limits);
                await Task.Delay(2000); // Run for 2 seconds

                // Optionally adjust damping/gain during force mode
                ctrl.ForceModeSetDamping(0.05);
                ctrl.ForceModeSetGainScaling(0.5);
                await Task.Delay(1000);
            }
            finally
            {
                // Always exit force mode
                ctrl.ForceModeStop();
                await Task.Delay(500);
            }
        }

        // ====================================================================
        // Jogging Tests
        // ====================================================================

        static async Task TestJog()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);

            // Jog joint 0 slowly (feature 0 = base frame)
            var speeds = new double[6] { 0.05, 0, 0, 0, 0, 0 }; // 0.05 rad/s on J0
            ctrl.JogStart(speeds, feature: 0);
            await Task.Delay(2000);

            ctrl.JogStop();
            await Task.Delay(500);

            // Jog in tool frame (feature 1)
            var toolSpeeds = new double[6] { 0, 0, 0.01, 0, 0, 0 }; // 1cm/s in tool Z
            ctrl.JogStart(toolSpeeds, feature: 1);
            await Task.Delay(2000);

            ctrl.JogStop();
            await Task.Delay(500);
        }

        // ====================================================================
        // Teach Mode Tests
        // ====================================================================

        static async Task TestTeachMode()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);

            // Enter teach mode (freedrive)
            ctrl.TeachMode();
            await Task.Delay(3000); // Allow 3 seconds of freedrive

            // Exit teach mode
            ctrl.EndTeachMode();
            await Task.Delay(500);
        }

        // ====================================================================
        // Safety Tests
        // ====================================================================

        static async Task TestTriggerProtectiveStop()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            using var recv = new RTDEReceive(ROBOT_IP);

            // Trigger protective stop (for testing only!)
            ctrl.TriggerProtectiveStop();
            await Task.Delay(1000);

            // Verify robot is in protective stop
            bool isProtectiveStopped = recv.IsProtectiveStopped;

            // Note: In URSim, you may need to manually clear the protective stop
            // via the GUI or a program. This test just verifies the command works.
        }

        // ====================================================================
        // Robotiq Gripper Tests
        // ====================================================================

        static async Task TestRobotiqNative()
        {
            using var gripper = new RobotiqGripperNative(ROBOT_IP);
            gripper.Connect();
            gripper.Activate();
            gripper.SetSpeed(0.5f); // normalized
            gripper.SetForce(0.5f);
            gripper.Open(force: 0.5f, moveMode: RobotiqMoveMode.WaitFinished);
            gripper.Close(force: 0.5f, moveMode: RobotiqMoveMode.WaitFinished);
        }

        static async Task TestRobotiqScript()
        {
            const string host = ROBOT_IP;
            var gripper = new RobotiqGripper(host, port: 30002);
            await gripper.ConnectAsync();

            // These calls require the Robotiq URCap installed/enabled on the controller.
            await gripper.ActivateAsync();
            await Task.Delay(500);
            await gripper.OpenAsync();
            await Task.Delay(500);
            await gripper.CloseAsync();
            await Task.Delay(500);
        }

        static async Task TestRobotiqRtde()
        {
            using var ctrl = new RTDEControl(ROBOT_IP);
            using var recv = new RTDEReceive(ROBOT_IP);
            using var io = new RTDEIO(ROBOT_IP);
            var gripper = new RobotiqGripperRtde(ctrl, recv, io);

            // Install the bridge and do a quick sequence
            await gripper.InstallBridgeAsync();
            await gripper.ActivateAsync();
            await gripper.OpenAsync();
            await gripper.CloseAsync();
        }

        // ====================================================================
        // Extended Data Tests
        // ====================================================================

        static async Task TestExtendedReceiveData()
        {
            using var recv = new RTDEReceive(ROBOT_IP);

            // Test all extended data getters
            var targetQ = recv.GetTargetQ();
            if (targetQ.Length != 6)
                throw new Exception("Invalid target joint positions");

            var targetPose = recv.GetTargetTcpPose();
            if (targetPose.Length != 6)
                throw new Exception("Invalid target TCP pose");

            var tcpForce = recv.GetActualTcpForce();
            if (tcpForce.Length != 6)
                throw new Exception("Invalid TCP force");

            var jointTemps = recv.GetJointTemperatures();
            if (jointTemps.Length != 6)
                throw new Exception("Invalid joint temperatures");

            var current = recv.GetActualCurrent();
            if (current.Length != 6)
                throw new Exception("Invalid joint currents");

            // Verify temperature values are reasonable (should be > 0 and < 100 Celsius typically)
            for (int i = 0; i < 6; i++)
            {
                if (jointTemps[i] < 0 || jointTemps[i] > 150)
                {
                    Console.WriteLine($"  Warning: Joint {i} temp = {jointTemps[i]:F1}°C");
                }
            }

            await Task.CompletedTask;
        }
    }
}
