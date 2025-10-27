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
        internal static extern Status ur_rtde_control_trigger_watchdog(IntPtr handle);

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
