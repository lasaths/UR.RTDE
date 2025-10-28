using System;
using UR.RTDE;

namespace URSimTests
{
    public static class QuickTest
    {
        public static void TestAdvancedFunctions()
        {
            Console.WriteLine("=== Testing Advanced Function Availability ===\n");
            
            try
            {
                // Test that we can create instances (this validates P/Invoke bindings exist)
                Console.Write("Testing RTDEControl instantiation... ");
                using var ctrl = new RTDEControl("192.168.1.999", frequency: -1.0); // Use fake IP to avoid hanging
                Console.WriteLine("✓ Class created");
                
                // Check if methods exist via reflection
                var ctrlType = typeof(RTDEControl);
                
                var methods = new[] {
                    "ServoL",
                    "ZeroFtSensor",
                    "ForceMode",
                    "ForceModeStop",
                    "ForceModeSetDamping",
                    "ForceModeSetGainScaling",
                    "JogStart",
                    "JogStop",
                    "TeachMode",
                    "EndTeachMode",
                    "TriggerProtectiveStop"
                };
                
                Console.WriteLine("\nChecking method availability:");
                int found = 0;
                foreach (var methodName in methods)
                {
                    var method = ctrlType.GetMethod(methodName);
                    if (method != null)
                    {
                        Console.WriteLine($"  ✓ {methodName}");
                        found++;
                    }
                    else
                    {
                        Console.WriteLine($"  ✗ {methodName} - NOT FOUND");
                    }
                }
                
                Console.WriteLine($"\nResult: {found}/{methods.Length} methods found");
                
                if (found == methods.Length)
                {
                    Console.WriteLine("\n✓ All advanced functions are available!");
                }
                else
                {
                    Console.WriteLine("\n✗ Some functions are missing");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"✗ Error: {ex.Message}");
                if (ex.InnerException != null)
                {
                    Console.WriteLine($"   Inner: {ex.InnerException.Message}");
                }
            }
        }
    }
}
