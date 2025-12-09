using System;
using System.Runtime.InteropServices;

namespace UR.RTDE.Native
{
    /// <summary>
    /// Low-level P/Invoke bindings to ur_rtde_c_api native library
    /// </summary>
    internal static class NativeMethods
    {
        private const string DllName = "ur_rtde_c_api";

        // ====================================================================
        // Enums and Constants
        // ====================================================================

        internal enum Status
        {
            OK = 0,
            ErrorConnection = -1,
            ErrorTimeout = -2,
            ErrorInvalidHandle = -3,
            ErrorInvalidParam = -4,
            ErrorNotConnected = -5,
            ErrorCommandFailed = -6,
            ErrorUnknown = -99
        }

        [Flags]
        internal enum Flags : ushort
        {
            UploadScript = 1 << 0,
            UseExtUrCap = 1 << 1,
            Verbose = 1 << 2,
            UpperRangeRegisters = 1 << 3,
            NoWait = 1 << 4,
            CustomScript = 1 << 5,
            NoExtFt = 1 << 6,
            DisableRemoteControlCheck = 1 << 7,
            Default = UploadScript
        }

        internal enum RobotiqMoveMode
        {
            StartMove = 0,
            WaitFinished = 1
        }

        internal enum RobotiqMoveParameter
        {
            Position = 0,
            Speed = 1,
            Force = 2
        }

        internal enum RobotiqUnit
        {
            Device = 0,
            Normalized = 1,
            Percent = 2,
            Millimeter = 3
        }

        internal enum RobotiqPositionId
        {
            Open = 0,
            Close = 1
        }

        // ====================================================================
        // Control Interface
        // ====================================================================

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern IntPtr ur_rtde_control_create(
            string hostname,
            double frequency,
            ushort flags,
            int ur_cap_port);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ur_rtde_control_destroy(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_control_is_connected(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_disconnect(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_reconnect(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_move_j(
            IntPtr handle,
            [In] double[] q,
            double speed,
            double acceleration,
            [MarshalAs(UnmanagedType.I1)] bool asynchronous);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_move_l(
            IntPtr handle,
            [In] double[] pose,
            double speed,
            double acceleration,
            [MarshalAs(UnmanagedType.I1)] bool asynchronous);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_stop_j(
            IntPtr handle,
            double acceleration,
            [MarshalAs(UnmanagedType.I1)] bool asynchronous);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_stop_l(
            IntPtr handle,
            double acceleration,
            [MarshalAs(UnmanagedType.I1)] bool asynchronous);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_speed_j(
            IntPtr handle,
            [In] double[] qd,
            double acceleration,
            double time);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_speed_l(
            IntPtr handle,
            [In] double[] xd,
            double acceleration,
            double time);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_servo_j(
            IntPtr handle,
            [In] double[] q,
            double speed,
            double acceleration,
            double time,
            double lookahead_time,
            double gain);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_set_tcp(
            IntPtr handle,
            [In] double[] tcp_pose);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_set_payload(
            IntPtr handle,
            double mass,
            [In] double[] cog);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_kick_watchdog(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern IntPtr ur_rtde_control_get_last_error(IntPtr handle);

        // ====================================================================
        // Receive Interface
        // ====================================================================

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern IntPtr ur_rtde_receive_create(
            string hostname,
            double frequency,
            ushort flags);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ur_rtde_receive_destroy(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_receive_is_connected(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_receive_get_actual_q(
            IntPtr handle,
            [Out] double[] q_out);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_receive_get_actual_tcp_pose(
            IntPtr handle,
            [Out] double[] pose_out);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_receive_get_actual_qd(
            IntPtr handle,
            [Out] double[] qd_out);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_receive_get_actual_tcp_speed(
            IntPtr handle,
            [Out] double[] speed_out);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ur_rtde_receive_get_robot_mode(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ur_rtde_receive_get_safety_mode(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ur_rtde_receive_get_runtime_state(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_receive_get_standard_digital_in(
            IntPtr handle,
            int index);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_receive_get_standard_digital_out(
            IntPtr handle,
            int index);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern IntPtr ur_rtde_receive_get_last_error(IntPtr handle);

        // ====================================================================
        // RTDEControl - Kinematics
        // ====================================================================

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_get_inverse_kinematics(
            IntPtr handle,
            [In] double[] x,
            UIntPtr x_size,
            [Out] double[] q_out,
            UIntPtr q_size);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_get_forward_kinematics(
            IntPtr handle,
            [In] double[] q,
            UIntPtr q_size,
            [Out] double[] x_out,
            UIntPtr x_size);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_control_get_inverse_kinematics_has_solution(
            IntPtr handle,
            [In] double[] x,
            UIntPtr x_size);

        // ====================================================================
        // RTDEControl - Advanced Movement
        // ====================================================================

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_servo_c(
            IntPtr handle,
            [In] double[] pose,
            double speed,
            double acceleration,
            double blend);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_servo_stop(
            IntPtr handle,
            double acceleration);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_speed_stop(
            IntPtr handle,
            double acceleration);

        // ====================================================================
        // RTDEControl - Safety & Status
        // ====================================================================

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_control_is_program_running(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_control_is_steady(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern uint ur_rtde_control_get_robot_status(IntPtr handle);

        // ====================================================================
        // RTDEReceive - Extended Data
        // ====================================================================

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_receive_get_target_q(
            IntPtr handle,
            [Out] double[] q_out,
            UIntPtr q_size);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_receive_get_target_tcp_pose(
            IntPtr handle,
            [Out] double[] pose_out,
            UIntPtr pose_size);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_receive_get_actual_tcp_force(
            IntPtr handle,
            [Out] double[] force_out,
            UIntPtr force_size);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_receive_get_joint_temperatures(
            IntPtr handle,
            [Out] double[] temps_out,
            UIntPtr temps_size);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_receive_get_actual_current(
            IntPtr handle,
            [Out] double[] current_out,
            UIntPtr current_size);

        // ====================================================================
        // RTDEReceive - Safety Status
        // ====================================================================

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_receive_is_protective_stopped(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_receive_is_emergency_stopped(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern uint ur_rtde_receive_get_robot_status(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern uint ur_rtde_receive_get_safety_status_bits(IntPtr handle);

        // ====================================================================
        // RTDEReceive - RTDE Output Registers
        // ====================================================================

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ur_rtde_receive_get_output_int_register(
            IntPtr handle,
            ushort reg);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double ur_rtde_receive_get_output_double_register(
            IntPtr handle,
            ushort reg);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_receive_get_output_bit_register(
            IntPtr handle,
            ushort reg);

        // ====================================================================
        // RTDEReceive - Analog I/O
        // ====================================================================

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double ur_rtde_receive_get_standard_analog_input(
            IntPtr handle,
            byte input_id);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double ur_rtde_receive_get_standard_analog_output(
            IntPtr handle,
            byte output_id);

        // ====================================================================
        // RTDEControl - Force Mode & Advanced Control
        // ====================================================================

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_force_mode(
            IntPtr handle,
            [In] double[] task_frame,
            UIntPtr task_frame_size,
            [In] int[] selection_vector,
            UIntPtr selection_vector_size,
            [In] double[] wrench,
            UIntPtr wrench_size,
            int type,
            [In] double[] limits,
            UIntPtr limits_size);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_force_mode_stop(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_zero_ft_sensor(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_jog_start(
            IntPtr handle,
            [In] double[] speeds,
            UIntPtr speeds_size,
            int feature);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_jog_stop(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_teach_mode(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_end_teach_mode(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_servo_l(
            IntPtr handle,
            [In] double[] pose,
            UIntPtr pose_size,
            double speed,
            double acceleration,
            double time,
            double lookahead_time,
            double gain);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_trigger_protective_stop(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_force_mode_set_damping(
            IntPtr handle,
            double damping);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_force_mode_set_gain_scaling(
            IntPtr handle,
            double scaling);

        // ====================================================================
        // RTDEControl - RTDE Registers & Custom Script
        // ====================================================================

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_set_input_int_register(
            IntPtr handle,
            ushort reg,
            int value);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_set_input_double_register(
            IntPtr handle,
            ushort reg,
            double value);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_control_set_input_bit_register(
            IntPtr handle,
            ushort reg,
            [MarshalAs(UnmanagedType.I1)] bool value);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern Status ur_rtde_control_send_custom_script(
            IntPtr handle,
            string script);

        // ====================================================================
        // RTDEIO Interface
        // ====================================================================

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern IntPtr ur_rtde_io_create(
            string hostname,
            ushort flags);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ur_rtde_io_destroy(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_io_is_connected(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_io_set_standard_digital_out(
            IntPtr handle,
            byte output_id,
            [MarshalAs(UnmanagedType.I1)] bool signal_level);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_io_set_tool_digital_out(
            IntPtr handle,
            byte output_id,
            [MarshalAs(UnmanagedType.I1)] bool signal_level);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_io_set_analog_output_voltage(
            IntPtr handle,
            byte output_id,
            double voltage_ratio);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_io_set_analog_output_current(
            IntPtr handle,
            byte output_id,
            double current_ratio);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_io_set_speed_slider(
            IntPtr handle,
            double speed);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ur_rtde_io_disconnect(IntPtr handle);

        // ====================================================================
        // Robotiq Gripper (robotiq_gripper.h)
        // ====================================================================

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern IntPtr ur_rtde_robotiq_gripper_create(
            string hostname,
            int port,
            [MarshalAs(UnmanagedType.I1)] bool verbose);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ur_rtde_robotiq_gripper_destroy(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_robotiq_gripper_connect(
            IntPtr handle,
            uint timeout_ms);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ur_rtde_robotiq_gripper_disconnect(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_robotiq_gripper_is_connected(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_robotiq_gripper_activate(
            IntPtr handle,
            [MarshalAs(UnmanagedType.I1)] bool auto_calibrate);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_robotiq_gripper_auto_calibrate(
            IntPtr handle,
            float speed);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_robotiq_gripper_is_active(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern float ur_rtde_robotiq_gripper_get_open_position(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern float ur_rtde_robotiq_gripper_get_closed_position(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern float ur_rtde_robotiq_gripper_get_current_position(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_robotiq_gripper_is_open(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ur_rtde_robotiq_gripper_is_closed(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ur_rtde_robotiq_gripper_move(
            IntPtr handle,
            float position,
            float speed,
            float force,
            int move_mode);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ur_rtde_robotiq_gripper_open(
            IntPtr handle,
            float speed,
            float force,
            int move_mode);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ur_rtde_robotiq_gripper_close(
            IntPtr handle,
            float speed,
            float force,
            int move_mode);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_robotiq_gripper_emergency_release(
            IntPtr handle,
            int direction,
            int move_mode);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ur_rtde_robotiq_gripper_fault_status(IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_robotiq_gripper_set_unit(
            IntPtr handle,
            int param,
            int unit);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_robotiq_gripper_set_position_range_mm(
            IntPtr handle,
            int range_mm);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern float ur_rtde_robotiq_gripper_set_speed(
            IntPtr handle,
            float speed);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern float ur_rtde_robotiq_gripper_set_force(
            IntPtr handle,
            float force);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ur_rtde_robotiq_gripper_object_detection_status(
            IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int ur_rtde_robotiq_gripper_wait_for_motion_complete(
            IntPtr handle);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern Status ur_rtde_robotiq_gripper_set_var(
            IntPtr handle,
            string name,
            int value);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern int ur_rtde_robotiq_gripper_get_var(
            IntPtr handle,
            string name);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern Status ur_rtde_robotiq_gripper_set_vars(
            IntPtr handle,
            string[] names,
            int[] values,
            UIntPtr count);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern Status ur_rtde_robotiq_gripper_get_vars(
            IntPtr handle,
            string[] names,
            UIntPtr count,
            [Out] int[] values);

        /// <summary>
        /// Managed wrapper for ur_rtde_robotiq_gripper_get_vars.
        /// </summary>
        internal static int[] RobotiqGripperGetVars(IntPtr handle, string[] names)
        {
            if (handle == IntPtr.Zero)
                throw new ArgumentNullException(nameof(handle));
            if (names == null || names.Length == 0)
                throw new ArgumentException("names array must not be null or empty", nameof(names));

            int[] values = new int[names.Length];
            Status status = ur_rtde_robotiq_gripper_get_vars(handle, names, (UIntPtr)names.Length, values);
            if (status != Status.OK)
                throw new InvalidOperationException($"Native call failed with status: {status}");
            return values;
        }
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_robotiq_gripper_get_native_position_range(
            IntPtr handle,
            out int min_position,
            out int max_position);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern Status ur_rtde_robotiq_gripper_set_native_position_range(
            IntPtr handle,
            int min_position,
            int max_position);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        internal static extern IntPtr ur_rtde_robotiq_gripper_get_last_error(IntPtr handle);

        /// <summary>
        /// Returns the last error string for the Robotiq gripper, marshaled to a managed string.
        /// </summary>
        internal static string? GetLastErrorString(IntPtr handle)
        {
            IntPtr ptr = ur_rtde_robotiq_gripper_get_last_error(handle);
            return GetString(ptr);
        }
        // ====================================================================
        // Helper Methods
        // ====================================================================

        internal static string? GetString(IntPtr ptr)
        {
            if (ptr == IntPtr.Zero)
                return null;
            return Marshal.PtrToStringAnsi(ptr);
        }
    }
}
