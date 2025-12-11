// This file implements a P/Invoke wrapper around native C++ code (ur_rtde::RTDEIO).
// All NativeMethods.* calls are intentional and required - this is the purpose of the wrapper.
// These calls cannot be replaced with managed code as they interface with native C++ libraries.

using System;
using UR.RTDE.Native;

namespace UR.RTDE
{
    /// <summary>
    /// RTDE I/O Interface for controlling robot outputs
    /// 
    /// This class uses P/Invoke to call native C++ functions. All NativeMethods.* calls are intentional
    /// and required for interfacing with the underlying ur_rtde C++ library.
    /// </summary>
    public sealed class RTDEIO : IDisposable
    {
        private IntPtr _handle;
        private bool _disposed;
        private readonly string _hostname;

        /// <summary>
        /// Create RTDE I/O Interface
        /// </summary>
        /// <param name="hostname">Robot IP address or hostname</param>
        /// <param name="verbose">Enable verbose logging</param>
        public RTDEIO(string hostname, bool verbose = false)
        {
            if (string.IsNullOrWhiteSpace(hostname))
                throw new ArgumentNullException(nameof(hostname));

            _hostname = hostname;
            ushort flags = verbose ? (ushort)NativeMethods.Flags.Verbose : (ushort)0;
            _handle = NativeMethods.ur_rtde_io_create(hostname, flags);

            if (_handle == IntPtr.Zero)
                throw new RTDEConnectionException($"Failed to connect to robot at {hostname}");
        }

        /// <summary>
        /// Check if connected to robot
        /// </summary>
        public bool IsConnected
        {
            get
            {
                if (_disposed) return false;
                return NativeMethods.ur_rtde_io_is_connected(_handle);
            }
        }

        /// <summary>
        /// Set standard digital output
        /// </summary>
        /// <param name="index">Output index (0-7)</param>
        /// <param name="value">Output value (true = HIGH, false = LOW)</param>
        public void SetStandardDigitalOut(int index, bool value)
        {
            ThrowIfDisposed();
            if (index < 0 || index > 7)
                throw new ArgumentOutOfRangeException(nameof(index), "Index must be between 0 and 7");
            
            CheckStatus(NativeMethods.ur_rtde_io_set_standard_digital_out(_handle, (byte)index, value));
        }

        /// <summary>
        /// Set tool digital output
        /// </summary>
        /// <param name="index">Output index (0-1)</param>
        /// <param name="value">Output value (true = HIGH, false = LOW)</param>
        public void SetToolDigitalOut(int index, bool value)
        {
            ThrowIfDisposed();
            if (index < 0 || index > 1)
                throw new ArgumentOutOfRangeException(nameof(index), "Index must be 0 or 1");
            
            CheckStatus(NativeMethods.ur_rtde_io_set_tool_digital_out(_handle, (byte)index, value));
        }

        /// <summary>
        /// Set analog output voltage
        /// </summary>
        /// <param name="index">Output index (0-1)</param>
        /// <param name="voltageRatio">Voltage ratio (0.0 to 1.0, represents 0V to 10V)</param>
        public void SetAnalogOutputVoltage(int index, double voltageRatio)
        {
            ThrowIfDisposed();
            if (index < 0 || index > 1)
                throw new ArgumentOutOfRangeException(nameof(index), "Index must be 0 or 1");
            if (voltageRatio < 0.0 || voltageRatio > 1.0)
                throw new ArgumentOutOfRangeException(nameof(voltageRatio), "Voltage ratio must be between 0.0 and 1.0");
            
            CheckStatus(NativeMethods.ur_rtde_io_set_analog_output_voltage(_handle, (byte)index, voltageRatio));
        }

        /// <summary>
        /// Set analog output current
        /// </summary>
        /// <param name="index">Output index (0-1)</param>
        /// <param name="currentRatio">Current ratio (0.0 to 1.0, represents 4mA to 20mA)</param>
        public void SetAnalogOutputCurrent(int index, double currentRatio)
        {
            ThrowIfDisposed();
            if (index < 0 || index > 1)
                throw new ArgumentOutOfRangeException(nameof(index), "Index must be 0 or 1");
            if (currentRatio < 0.0 || currentRatio > 1.0)
                throw new ArgumentOutOfRangeException(nameof(currentRatio), "Current ratio must be between 0.0 and 1.0");
            
            CheckStatus(NativeMethods.ur_rtde_io_set_analog_output_current(_handle, (byte)index, currentRatio));
        }

        /// <summary>
        /// Write an integer to an RTDE input register.
        /// </summary>
        /// <param name="reg">Register index (0-47; ranges are validated by firmware)</param>
        /// <param name="value">Integer value to write</param>
        public void SetInputIntRegister(int reg, int value)
        {
            ThrowIfDisposed();
            if (reg < 0 || reg > ushort.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(reg), "Register index must be non-negative");

            CheckStatus(NativeMethods.ur_rtde_io_set_input_int_register(_handle, (ushort)reg, value));
        }

        /// <summary>
        /// Write a double to an RTDE input register.
        /// </summary>
        /// <param name="reg">Register index (0-47; ranges are validated by firmware)</param>
        /// <param name="value">Double value to write</param>
        public void SetInputDoubleRegister(int reg, double value)
        {
            ThrowIfDisposed();
            if (reg < 0 || reg > ushort.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(reg), "Register index must be non-negative");

            CheckStatus(NativeMethods.ur_rtde_io_set_input_double_register(_handle, (ushort)reg, value));
        }

        /// <summary>
        /// Set speed slider override (runtime speed control)
        /// </summary>
        /// <param name="speed">Speed fraction (0.0 to 1.0, where 1.0 = 100% speed)</param>
        public void SetSpeedSlider(double speed)
        {
            ThrowIfDisposed();
            if (speed < 0.0 || speed > 1.0)
                throw new ArgumentOutOfRangeException(nameof(speed), "Speed must be between 0.0 and 1.0");
            
            CheckStatus(NativeMethods.ur_rtde_io_set_speed_slider(_handle, speed));
        }

        /// <summary>
        /// Disconnect from robot
        /// </summary>
        public void Disconnect()
        {
            if (_disposed || _handle == IntPtr.Zero)
                return;

            NativeMethods.ur_rtde_io_disconnect(_handle);
        }

        private void CheckStatus(NativeMethods.Status status)
        {
            if (status == NativeMethods.Status.OK)
                return;

            throw status switch
            {
                NativeMethods.Status.ErrorConnection => new RTDEConnectionException($"Connection error on {_hostname}"),
                NativeMethods.Status.ErrorTimeout => new RTDEException("Operation timed out"),
                NativeMethods.Status.ErrorNotConnected => new RTDEConnectionException($"Not connected to {_hostname}"),
                NativeMethods.Status.ErrorCommandFailed => new RTDEException("I/O command failed"),
                _ => new RTDEException($"Error {status}")
            };
        }

        private void ThrowIfDisposed()
        {
            if (_disposed)
                throw new ObjectDisposedException(nameof(RTDEIO));
        }

        public void Dispose()
        {
            if (_disposed)
                return;

            Disconnect();

            if (_handle != IntPtr.Zero)
            {
                NativeMethods.ur_rtde_io_destroy(_handle);
                _handle = IntPtr.Zero;
            }

            _disposed = true;
        }
    }
}
