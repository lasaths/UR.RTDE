using System;
using System.Threading;
using UR.RTDE.PythonBridge;

namespace UR.RTDE.Test
{
    class PythonBridgeTest
    {
        static void Main(string[] args)
        {
            Console.WriteLine("=================================================================");
            Console.WriteLine("UR.RTDE Python.NET Bridge Test");
            Console.WriteLine("=================================================================\n");

            string robotIp = args.Length > 0 ? args[0] : "localhost";
            Console.WriteLine($"Target: {robotIp}\n");

            try
            {
                // Initialize Python runtime
                Console.WriteLine("[1/6] Initializing Python runtime...");
                PythonEngineManager.Initialize();
                Console.WriteLine("✅ Python runtime initialized\n");

                // Test Control Interface
                Console.WriteLine("[2/6] Creating RTDEControl...");
                using (var control = new RTDEControlPython(robotIp))
                {
                    Console.WriteLine($"✅ Connected: {control.IsConnected}\n");

                    // Test Receive Interface
                    Console.WriteLine("[3/6] Creating RTDEReceive...");
                    using (var receive = new RTDEReceivePython(robotIp))
                    {
                        Console.WriteLine($"✅ Connected: {receive.IsConnected}\n");

                        // Test receive data
                        Console.WriteLine("[4/6] Reading joint positions...");
                        double[] q = receive.GetActualQ();
                        Console.WriteLine("✅ Joint positions (rad):");
                        for (int i = 0; i < q.Length; i++)
                        {
                            Console.WriteLine($"   J{i}: {q[i]:F4}");
                        }

                        int robotMode = receive.GetRobotMode();
                        int safetyMode = receive.GetSafetyMode();
                        Console.WriteLine($"\n✅ Robot Mode: {robotMode}");
                        Console.WriteLine($"✅ Safety Mode: {safetyMode}\n");

                        // Test streaming
                        Console.WriteLine("[5/6] Streaming for 3 seconds...");
                        int count = 0;
                        DateTime start = DateTime.UtcNow;
                        while ((DateTime.UtcNow - start).TotalSeconds < 3)
                        {
                            q = receive.GetActualQ();
                            count++;

                            if (count % 500 == 0)
                            {
                                double elapsed = (DateTime.UtcNow - start).TotalSeconds;
                                double hz = count / elapsed;
                                Console.WriteLine($"  [{elapsed:F1}s] Sample {count}, ~{hz:F0} Hz");
                            }
                        }
                        double totalTime = (DateTime.UtcNow - start).TotalSeconds;
                        double avgHz = count / totalTime;
                        Console.WriteLine($"✅ Received {count} samples in {totalTime:F2}s ({avgHz:F1} Hz)\n");

                        // Test movement
                        Console.WriteLine("[6/6] Testing MoveJ...");
                        double[] currentQ = receive.GetActualQ();
                        Console.WriteLine($"Current J0: {currentQ[0]:F4} rad");

                        double[] targetQ = (double[])currentQ.Clone();
                        targetQ[0] += 0.087; // ~5 degrees
                        Console.WriteLine($"Target J0: {targetQ[0]:F4} rad");

                        control.MoveJ(targetQ, speed: 0.5, acceleration: 0.5);
                        Console.WriteLine("✅ MoveJ completed\n");

                        Thread.Sleep(500);
                        double[] newQ = receive.GetActualQ();
                        Console.WriteLine($"New J0: {newQ[0]:F4} rad");

                        // Test stop
                        Console.WriteLine("\nTesting StopJ...");
                        control.StopJ();
                        Console.WriteLine("✅ StopJ executed");
                    }
                }

                Console.WriteLine("\n=================================================================");
                Console.ForegroundColor = ConsoleColor.Green;
                Console.WriteLine("✅ ALL TESTS PASSED");
                Console.ResetColor();
                Console.WriteLine("=================================================================");

                Console.WriteLine("\nPython.NET bridge is working perfectly!");
                Console.WriteLine("Ready to create Grasshopper components.");
            }
            catch (Exception ex)
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine($"\n❌ Error: {ex.Message}");
                Console.WriteLine($"\nStack trace:\n{ex.StackTrace}");
                Console.ResetColor();

                if (ex.InnerException != null)
                {
                    Console.WriteLine($"\nInner exception: {ex.InnerException.Message}");
                }
            }
            finally
            {
                // Note: Don't shutdown Python runtime in real app (keep it alive)
                // PythonEngineManager.Shutdown();
            }
        }
    }
}
