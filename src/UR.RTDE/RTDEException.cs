using System;

namespace UR.RTDE
{
    /// <summary>
    /// Exception thrown when RTDE operations fail
    /// </summary>
    public class RTDEException : Exception
    {
        public RTDEException(string message) : base(message) { }
        public RTDEException(string message, Exception inner) : base(message, inner) { }
    }

    /// <summary>
    /// Exception thrown when connection to robot fails
    /// </summary>
    public class RTDEConnectionException : RTDEException
    {
        public RTDEConnectionException(string message) : base(message) { }
        public RTDEConnectionException(string message, Exception inner) : base(message, inner) { }
    }
}
