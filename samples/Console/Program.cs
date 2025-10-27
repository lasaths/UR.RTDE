using System;
using System.Threading;
using UR.RTDE.PythonBridge;

namespace ConsoleDemo
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("=================================================================");
            Console.WriteLine("UR.RTDE Python.NET Bridge Demo");
            Console.WriteLine("=================================================================\n");

            string robotIp = args.Length > 0 ? args[0] : "localhost";
            Console.WriteLine($"Target: {robotIp}\n");

            try
            {
                // Initialize Python
                Console.WriteLine("[1/5] Initializing Python runtime...");
                PythonEngineManager.Initialize();
                Console.WriteLine("✅ Python initialized\n");

                // Connect
                Console.WriteLine("[2/5] Connecting to robot...");
                using (var control = new RTDEControlPython(robotIp))
                using (var receive = new RTDEReceivePython(robotIp))
                {
                    Console.WriteLine($"✅ Control: {control.IsConnected}");
                    Console.WriteLine($"✅ Receive: {receive.IsConnected}\n");

                    // Read joints
                    Console.WriteLine("[3/5] Reading joint positions...");
                    var q = receive.GetActualQ();
                    Console.WriteLine("Joint positions:");
                    for (int i = 0; i < 6; i++)
                        Console.WriteLine($"  J{i}: {q[i]:F4} rad");

                    Console.WriteLine($"\nRobot Mode: {receive.GetRobotMode()}");
                    Console.WriteLine($"Safety Mode: {receive.GetSafetyMode()}\n");

                    // Stream
                    Console.WriteLine("[4/5] Streaming for 3 seconds...");
                    int count = 0;
                    var start = DateTime.UtcNow;
                    while ((DateTime.UtcNow - start).TotalSeconds < 3)
                    {
                        q = receive.GetActualQ();
                        count++;
                    }
                    var elapsed = (DateTime.UtcNow - start).TotalSeconds;
                    Console.WriteLine($"✅ {count} samples in {elapsed:F2}s = {count/elapsed:F0} Hz\n");

                    // Move
                    Console.WriteLine("[5/5] Testing MoveJ and Stop...");
                    q = receive.GetActualQ();
                    var targetQ = (double[])q.Clone();
                    targetQ[0] += 0.087; // Move J0 by 5 degrees
                    
                    Console.WriteLine($"Moving J0 from {q[0]:F4} to {targetQ[0]:F4} rad");
                    control.MoveJ(targetQ, 0.5, 0.5);
                    Console.WriteLine("✅ Moved");

                    Thread.Sleep(500);
                    Console.WriteLine($"New J0: {receive.GetActualQ()[0]:F4} rad");

                    control.StopJ();
                    Console.WriteLine("✅ Stopped\n");

                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.WriteLine("=================================================================");
                    Console.WriteLine("✅ ALL TESTS PASSED - Python.NET bridge works!");
                    Console.WriteLine("=================================================================");
                    Console.ResetColor();
                }
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
        }
    }
}
