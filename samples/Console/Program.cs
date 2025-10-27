using System;
using System.Threading;
using UR.RTDE;

namespace ConsoleDemo
{
    class Program
    {
        static void Main(string[] args)
        {
            if (args.Length < 1)
            {
                Console.WriteLine("Usage: ConsoleDemo <robot_ip>");
                Console.WriteLine("Example: ConsoleDemo 192.168.1.100");
                return;
            }

            string robotIp = args[0];
            Console.WriteLine($"UR.RTDE Console Demo");
            Console.WriteLine($"Connecting to robot at {robotIp}...\n");

            try
            {
                // Test 1: Control Interface
                Console.WriteLine("=== Test 1: Control Interface ===");
                using (var control = new RTDEControl(robotIp))
                {
                    Console.WriteLine($"✓ Connected: {control.IsConnected}");

                    // Get current position via receive interface
                    using (var receive = new RTDEReceive(robotIp))
                    {
                        var currentQ = receive.GetActualQ();
                        Console.WriteLine($"Current joint positions (rad):");
                        for (int i = 0; i < currentQ.Length; i++)
                        {
                            Console.WriteLine($"  J{i}: {currentQ[i]:F4}");
                        }

                        // Move to a nearby position (10 degrees on J0)
                        Console.WriteLine("\nMoving J0 by 10 degrees...");
                        var targetQ = (double[])currentQ.Clone();
                        targetQ[0] += 10.0 * Math.PI / 180.0; // Convert degrees to radians

                        control.MoveJ(targetQ, speed: 0.5, acceleration: 0.5, asynchronous: false);
                        Console.WriteLine("✓ Move completed");

                        // Stop
                        Console.WriteLine("\nStopping robot...");
                        control.StopJ(acceleration: 2.0);
                        Console.WriteLine("✓ Stop completed");
                    }
                }

                // Test 2: Receive Interface with Streaming
                Console.WriteLine("\n=== Test 2: Streaming Joint Data (5 seconds) ===");
                using (var receive = new RTDEReceive(robotIp))
                {
                    int updateCount = 0;
                    receive.StateReceived += (sender, state) =>
                    {
                        updateCount++;
                        if (updateCount % 100 == 0) // Print every 100th update
                        {
                            Console.WriteLine($"[{state.Timestamp:HH:mm:ss.fff}] J0={state.ActualQ[0]:F4} rad, " +
                                            $"Mode={state.RobotMode}, Safety={state.SafetyMode}");
                        }
                    };

                    receive.StartReceiving(updateRateMs: 2); // 500 Hz
                    Thread.Sleep(5000); // Stream for 5 seconds
                    receive.StopReceiving();

                    Console.WriteLine($"✓ Received {updateCount} updates");
                }

                Console.WriteLine("\n✅ All tests passed!");
            }
            catch (RTDEConnectionException ex)
            {
                Console.WriteLine($"❌ Connection error: {ex.Message}");
                Console.WriteLine("Make sure the robot is powered on and reachable.");
            }
            catch (RTDEException ex)
            {
                Console.WriteLine($"❌ RTDE error: {ex.Message}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"❌ Unexpected error: {ex.Message}");
                Console.WriteLine(ex.StackTrace);
            }
        }
    }
}
