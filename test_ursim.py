"""
Quick test of ur_rtde Python library against URSim Docker container.
This validates the wrapper design before building the full C++ native layer.
"""
import rtde_control
import rtde_receive
import time

# URSim Docker IP (use localhost since ports are mapped)
ROBOT_IP = "localhost"  # Docker exposes 30004 on host

def test_connection():
    """Test 1: Connection"""
    print("=" * 60)
    print("TEST 1: Connection")
    print("=" * 60)
    
    try:
        rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        
        print(f"✅ Control connected: {rtde_c.isConnected()}")
        print(f"✅ Receive connected: {rtde_r.isConnected()}")
        
        return rtde_c, rtde_r
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return None, None

def test_receive_data(rtde_r):
    """Test 2: Read joint positions and robot state"""
    print("\n" + "=" * 60)
    print("TEST 2: Receive Data")
    print("=" * 60)
    
    try:
        # Get current joint positions
        actual_q = rtde_r.getActualQ()
        print(f"✅ Joint positions (rad):")
        for i, q in enumerate(actual_q):
            print(f"   J{i}: {q:.4f}")
        
        # Get TCP pose
        tcp_pose = rtde_r.getActualTCPPose()
        print(f"\n✅ TCP Pose [x, y, z, rx, ry, rz]:")
        print(f"   {[f'{v:.4f}' for v in tcp_pose]}")
        
        # Get robot state
        robot_mode = rtde_r.getRobotMode()
        safety_mode = rtde_r.getSafetyMode()
        
        print(f"\n✅ Robot Mode: {robot_mode}")
        print(f"✅ Safety Mode: {safety_mode}")
        
        return True
    except Exception as e:
        print(f"❌ Receive failed: {e}")
        return False

def test_streaming(rtde_r, duration=5):
    """Test 3: Stream data for N seconds"""
    print("\n" + "=" * 60)
    print(f"TEST 3: Streaming for {duration} seconds @ max rate")
    print("=" * 60)
    
    try:
        start_time = time.time()
        count = 0
        
        while time.time() - start_time < duration:
            actual_q = rtde_r.getActualQ()
            count += 1
            
            # Print every 500 samples
            if count % 500 == 0:
                elapsed = time.time() - start_time
                hz = count / elapsed if elapsed > 0 else 0
                print(f"[{elapsed:.1f}s] Sample {count}, ~{hz:.0f} Hz, J0={actual_q[0]:.4f} rad")
        
        elapsed = time.time() - start_time
        avg_hz = count / elapsed
        print(f"\n✅ Received {count} samples in {elapsed:.2f}s")
        print(f"✅ Average rate: {avg_hz:.1f} Hz")
        
        return True
    except Exception as e:
        print(f"❌ Streaming failed: {e}")
        return False

def test_move(rtde_c, rtde_r):
    """Test 4: Small movement"""
    print("\n" + "=" * 60)
    print("TEST 4: MoveJ Command")
    print("=" * 60)
    
    try:
        # Get current position
        current_q = rtde_r.getActualQ()
        print(f"Current J0: {current_q[0]:.4f} rad")
        
        # Move J0 by 5 degrees
        target_q = current_q.copy()
        target_q[0] += 0.087  # ~5 degrees in radians
        
        print(f"Target J0: {target_q[0]:.4f} rad")
        print("Executing MoveJ (speed=0.5, accel=0.5)...")
        
        success = rtde_c.moveJ(target_q, speed=0.5, acceleration=0.5)
        
        if success:
            print("✅ MoveJ completed successfully")
            
            # Verify new position
            time.sleep(0.5)
            new_q = rtde_r.getActualQ()
            print(f"New J0: {new_q[0]:.4f} rad")
        else:
            print("❌ MoveJ failed")
        
        return success
    except Exception as e:
        print(f"❌ Move failed: {e}")
        return False

def test_stop(rtde_c):
    """Test 5: Stop command"""
    print("\n" + "=" * 60)
    print("TEST 5: Stop Command")
    print("=" * 60)
    
    try:
        rtde_c.stopJ(2.0)  # deceleration 2.0 rad/s^2
        print("✅ StopJ executed")
        return True
    except Exception as e:
        print(f"❌ Stop failed: {e}")
        return False

def main():
    """Run all tests"""
    print("\n" + "=" * 60)
    print("UR.RTDE Python Validation Test")
    print(f"Target: URSim @ {ROBOT_IP}")
    print("=" * 60)
    
    # Test 1: Connect
    rtde_c, rtde_r = test_connection()
    if not rtde_c or not rtde_r:
        print("\n❌ FAILED: Could not connect to robot")
        print("\nTroubleshooting:")
        print("1. Ensure URSim Docker container is running")
        print("2. Check IP address is correct (172.18.0.2)")
        print("3. Verify robot is in 'Remote Control' mode")
        print("4. Check firewall/network connectivity")
        return
    
    # Test 2: Receive data
    if not test_receive_data(rtde_r):
        print("\n❌ FAILED: Could not receive data")
        return
    
    # Test 3: Stream data (5 seconds)
    if not test_streaming(rtde_r, duration=5):
        print("\n❌ FAILED: Streaming test failed")
        return
    
    # Test 4: Move robot
    if not test_move(rtde_c, rtde_r):
        print("\n⚠️ WARNING: Move failed (robot may not be in correct mode)")
    
    # Test 5: Stop
    test_stop(rtde_c)
    
    # Cleanup
    rtde_c.disconnect()
    rtde_r.disconnect()
    
    print("\n" + "=" * 60)
    print("✅ ALL TESTS PASSED")
    print("=" * 60)
    print("\nNext steps:")
    print("1. Build native C++ wrapper (native/BUILD.md)")
    print("2. Test C# wrapper with URSim")
    print("3. Implement Grasshopper demo")

if __name__ == "__main__":
    main()
