// ur_rtde_c_api.h
// Thin C ABI façade over ur_rtde C++ library
// Purpose: Provide stable ABI for P/Invoke from C#
// Design: Opaque handles, C arrays, int status codes, no exceptions across boundary

#ifndef UR_RTDE_C_API_H
#define UR_RTDE_C_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Platform-specific export
#ifdef _WIN32
  #ifdef UR_RTDE_C_API_EXPORTS
    #define UR_RTDE_API __declspec(dllexport)
  #else
    #define UR_RTDE_API __declspec(dllimport)
  #endif
#else
  #define UR_RTDE_API __attribute__((visibility("default")))
#endif

// ============================================================================
// Error Codes
// ============================================================================
typedef enum {
  UR_RTDE_OK = 0,
  UR_RTDE_ERROR_CONNECTION = -1,
  UR_RTDE_ERROR_TIMEOUT = -2,
  UR_RTDE_ERROR_INVALID_HANDLE = -3,
  UR_RTDE_ERROR_INVALID_PARAM = -4,
  UR_RTDE_ERROR_NOT_CONNECTED = -5,
  UR_RTDE_ERROR_COMMAND_FAILED = -6,
  UR_RTDE_ERROR_UNKNOWN = -99
} ur_rtde_status_t;

// ============================================================================
// Opaque Handles (forward declarations)
// ============================================================================
typedef struct ur_rtde_control_handle ur_rtde_control_t;
typedef struct ur_rtde_receive_handle ur_rtde_receive_t;
typedef struct ur_rtde_robotiq_gripper ur_rtde_robotiq_gripper_t;

// ============================================================================
// Configuration Flags (matching RTDEControlInterface::Flags)
// ============================================================================
typedef enum {
  UR_RTDE_FLAG_UPLOAD_SCRIPT = 1 << 0,
  UR_RTDE_FLAG_USE_EXT_UR_CAP = 1 << 1,
  UR_RTDE_FLAG_VERBOSE = 1 << 2,
  UR_RTDE_FLAG_UPPER_RANGE_REGISTERS = 1 << 3,
  UR_RTDE_FLAG_NO_WAIT = 1 << 4,
  UR_RTDE_FLAG_CUSTOM_SCRIPT = 1 << 5,
  UR_RTDE_FLAG_NO_EXT_FT = 1 << 6,
  UR_RTDE_FLAG_DISABLE_REMOTE_CONTROL_CHECK = 1 << 7,
  UR_RTDE_FLAGS_DEFAULT = UR_RTDE_FLAG_UPLOAD_SCRIPT
} ur_rtde_flags_t;

// ============================================================================
// Control Interface
// ============================================================================

/**
 * Create RTDEControlInterface
 * @param hostname Robot IP address or hostname
 * @param frequency RTDE frequency (-1.0 for default: 500Hz e-Series, 125Hz CB)
 * @param flags Configuration flags (bitwise OR)
 * @param ur_cap_port External URCap port (default: 50002)
 * @return Opaque handle or NULL on failure
 */
UR_RTDE_API ur_rtde_control_t* ur_rtde_control_create(
  const char* hostname,
  double frequency,
  uint16_t flags,
  int ur_cap_port
);

/**
 * Destroy control interface and free resources
 */
UR_RTDE_API void ur_rtde_control_destroy(ur_rtde_control_t* handle);

/**
 * Check connection status
 */
UR_RTDE_API bool ur_rtde_control_is_connected(ur_rtde_control_t* handle);

/**
 * Wait until the next RTDE state has been received.
 * @return true if a new state arrived, false on error/timeout
 */
UR_RTDE_API bool ur_rtde_control_wait_for_next_state(ur_rtde_control_t* handle);

/**
 * Disconnect from robot
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_control_disconnect(ur_rtde_control_t* handle);

/**
 * Reconnect to robot
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_control_reconnect(ur_rtde_control_t* handle);

/**
 * MoveJ - Move to joint position (linear in joint-space)
 * @param handle Control handle
 * @param q Joint positions [6] in radians
 * @param speed Joint speed of leading axis [rad/s]
 * @param acceleration Joint acceleration [rad/s^2]
 * @param asynchronous Non-blocking if true
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_control_move_j(
  ur_rtde_control_t* handle,
  const double q[6],
  double speed,
  double acceleration,
  bool asynchronous
);

/**
 * MoveL - Move to pose (linear in tool-space)
 * @param handle Control handle
 * @param pose Target pose [x, y, z, rx, ry, rz] (axis-angle)
 * @param speed Tool speed [m/s]
 * @param acceleration Tool acceleration [m/s^2]
 * @param asynchronous Non-blocking if true
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_control_move_l(
  ur_rtde_control_t* handle,
  const double pose[6],
  double speed,
  double acceleration,
  bool asynchronous
);

/**
 * StopJ - Stop robot (linear in joint-space)
 * @param handle Control handle
 * @param acceleration Deceleration [rad/s^2]
 * @param asynchronous Non-blocking if true
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_control_stop_j(
  ur_rtde_control_t* handle,
  double acceleration,
  bool asynchronous
);

/**
 * StopL - Stop robot (linear in tool-space)
 * @param handle Control handle
 * @param acceleration Deceleration [m/s^2]
 * @param asynchronous Non-blocking if true
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_control_stop_l(
  ur_rtde_control_t* handle,
  double acceleration,
  bool asynchronous
);

/**
 * SpeedJ - Joint speed control
 * @param handle Control handle
 * @param qd Joint speeds [6] in rad/s
 * @param acceleration Joint acceleration [rad/s^2]
 * @param time Duration [s] (0 = indefinite)
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_control_speed_j(
  ur_rtde_control_t* handle,
  const double qd[6],
  double acceleration,
  double time
);

/**
 * SpeedL - Tool speed control
 * @param handle Control handle
 * @param xd Tool speed [6] in m/s and rad/s
 * @param acceleration Tool acceleration [m/s^2]
 * @param time Duration [s] (0 = indefinite)
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_control_speed_l(
  ur_rtde_control_t* handle,
  const double xd[6],
  double acceleration,
  double time
);

/**
 * ServoJ - Servo to joint position
 * @param handle Control handle
 * @param q Joint positions [6] in radians
 * @param speed NOT used
 * @param acceleration NOT used
 * @param time Command duration [s]
 * @param lookahead_time Smoothing [0.03-0.2 s]
 * @param gain Proportional gain [100-2000]
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_control_servo_j(
  ur_rtde_control_t* handle,
  const double q[6],
  double speed,
  double acceleration,
  double time,
  double lookahead_time,
  double gain
);

/**
 * SetTcp - Set tool center point
 * @param handle Control handle
 * @param tcp_pose TCP offset [x, y, z, rx, ry, rz]
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_control_set_tcp(
  ur_rtde_control_t* handle,
  const double tcp_pose[6]
);

/**
 * SetPayload - Set payload mass and CoG
 * @param handle Control handle
 * @param mass Payload mass [kg]
 * @param cog Center of gravity [x, y, z] in meters
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_control_set_payload(
  ur_rtde_control_t* handle,
  double mass,
  const double cog[3]
);

/**
 * KickWatchdog - Reset watchdog timer
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_control_kick_watchdog(
  ur_rtde_control_t* handle
);

// ============================================================================
// Receive Interface
// ============================================================================

/**
 * Create RTDEReceiveInterface
 * @param hostname Robot IP address
 * @param frequency RTDE frequency (-1.0 for default)
 * @param flags Configuration flags
 * @return Opaque handle or NULL on failure
 */
UR_RTDE_API ur_rtde_receive_t* ur_rtde_receive_create(
  const char* hostname,
  double frequency,
  uint16_t flags
);

/**
 * Destroy receive interface
 */
UR_RTDE_API void ur_rtde_receive_destroy(ur_rtde_receive_t* handle);

/**
 * Check connection status
 */
UR_RTDE_API bool ur_rtde_receive_is_connected(ur_rtde_receive_t* handle);

/**
 * Get actual joint positions
 * @param handle Receive handle
 * @param q_out Output buffer [6] in radians
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_receive_get_actual_q(
  ur_rtde_receive_t* handle,
  double q_out[6]
);

/**
 * Get actual TCP pose
 * @param handle Receive handle
 * @param pose_out Output buffer [x, y, z, rx, ry, rz]
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_receive_get_actual_tcp_pose(
  ur_rtde_receive_t* handle,
  double pose_out[6]
);

/**
 * Get actual joint speeds
 * @param handle Receive handle
 * @param qd_out Output buffer [6] in rad/s
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_receive_get_actual_qd(
  ur_rtde_receive_t* handle,
  double qd_out[6]
);

/**
 * Get actual TCP speed
 * @param handle Receive handle
 * @param speed_out Output buffer [6]
 */
UR_RTDE_API ur_rtde_status_t ur_rtde_receive_get_actual_tcp_speed(
  ur_rtde_receive_t* handle,
  double speed_out[6]
);

/**
 * Get robot mode (from RTDE)
 * @return Robot mode code
 */
UR_RTDE_API int32_t ur_rtde_receive_get_robot_mode(ur_rtde_receive_t* handle);

/**
 * Get safety mode
 * @return Safety mode code
 */
UR_RTDE_API int32_t ur_rtde_receive_get_safety_mode(ur_rtde_receive_t* handle);

/**
 * Get runtime state
 * @return Runtime state code
 */
UR_RTDE_API int32_t ur_rtde_receive_get_runtime_state(ur_rtde_receive_t* handle);

/**
 * Get standard digital input state
 * @param handle Receive handle
 * @param index Digital input index [0-7]
 * @return true if high, false if low or invalid
 */
UR_RTDE_API bool ur_rtde_receive_get_standard_digital_in(
  ur_rtde_receive_t* handle,
  int index
);

/**
 * Get standard digital output state
 * @param handle Receive handle
 * @param index Digital output index [0-7]
 * @return true if high, false if low or invalid
 */
UR_RTDE_API bool ur_rtde_receive_get_standard_digital_out(
  ur_rtde_receive_t* handle,
  int index
);

// ============================================================================
// Error Handling
// ============================================================================

/**
 * Get last error message for control interface
 */
UR_RTDE_API const char* ur_rtde_control_get_last_error(ur_rtde_control_t* handle);

/**
 * Get last error message for receive interface
 */
UR_RTDE_API const char* ur_rtde_receive_get_last_error(ur_rtde_receive_t* handle);

// ============================================================================
// RTDEControl - Kinematics
// ============================================================================

UR_RTDE_API ur_rtde_status_t ur_rtde_control_get_inverse_kinematics(
    ur_rtde_control_t* handle,
    const double* x,
    size_t x_size,
    double* q_out,
    size_t q_size);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_get_forward_kinematics(
    ur_rtde_control_t* handle,
    const double* q,
    size_t q_size,
    double* x_out,
    size_t x_size);

UR_RTDE_API bool ur_rtde_control_get_inverse_kinematics_has_solution(
    ur_rtde_control_t* handle,
    const double* x,
    size_t x_size);

// ============================================================================
// RTDEControl - Dynamics & Jacobians
// ============================================================================

UR_RTDE_API ur_rtde_status_t ur_rtde_control_direct_torque(
    ur_rtde_control_t* handle,
    const double* torque,
    size_t torque_size,
    bool friction_comp);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_get_mass_matrix(
    ur_rtde_control_t* handle,
    const double* q,
    size_t q_size,
    bool include_rotors_inertia,
    double* matrix_out,
    size_t matrix_size);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_get_coriolis_and_centrifugal_torques(
    ur_rtde_control_t* handle,
    const double* q,
    size_t q_size,
    const double* qd,
    size_t qd_size,
    double* torques_out,
    size_t torques_size);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_get_target_joint_accelerations(
    ur_rtde_control_t* handle,
    double* accelerations_out,
    size_t accelerations_size);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_get_jacobian(
    ur_rtde_control_t* handle,
    const double* pos,
    size_t pos_size,
    const double* tcp,
    size_t tcp_size,
    double* jacobian_out,
    size_t jacobian_size);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_get_jacobian_time_derivative(
    ur_rtde_control_t* handle,
    const double* pos,
    size_t pos_size,
    const double* vel,
    size_t vel_size,
    const double* tcp,
    size_t tcp_size,
    double* jacobian_out,
    size_t jacobian_size);

// ============================================================================
// RTDEControl - Additional Movement
// ============================================================================

UR_RTDE_API ur_rtde_status_t ur_rtde_control_servo_c(
    ur_rtde_control_t* handle,
    const double* pose,
    size_t pose_size,
    double speed,
    double acceleration,
    double blend);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_servo_stop(
    ur_rtde_control_t* handle,
    double acceleration);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_speed_stop(
    ur_rtde_control_t* handle,
    double acceleration);

// ============================================================================
// RTDEControl - Safety & Status
// ============================================================================

UR_RTDE_API bool ur_rtde_control_is_program_running(ur_rtde_control_t* handle);

UR_RTDE_API bool ur_rtde_control_is_steady(ur_rtde_control_t* handle);

UR_RTDE_API uint32_t ur_rtde_control_get_robot_status(ur_rtde_control_t* handle);

// ============================================================================
// RTDEReceive - Extended Data
// ============================================================================

UR_RTDE_API ur_rtde_status_t ur_rtde_receive_get_target_q(
    ur_rtde_receive_t* handle,
    double* q_out,
    size_t q_size);

UR_RTDE_API ur_rtde_status_t ur_rtde_receive_get_target_tcp_pose(
    ur_rtde_receive_t* handle,
    double* pose_out,
    size_t pose_size);

UR_RTDE_API ur_rtde_status_t ur_rtde_receive_get_actual_tcp_force(
    ur_rtde_receive_t* handle,
    double* force_out,
    size_t force_size);

UR_RTDE_API ur_rtde_status_t ur_rtde_receive_get_joint_temperatures(
    ur_rtde_receive_t* handle,
    double* temps_out,
    size_t temps_size);

UR_RTDE_API ur_rtde_status_t ur_rtde_receive_get_actual_current(
    ur_rtde_receive_t* handle,
    double* current_out,
    size_t current_size);

UR_RTDE_API ur_rtde_status_t ur_rtde_receive_get_actual_current_as_torque(
    ur_rtde_receive_t* handle,
    double* torque_out,
    size_t torque_size);

// ============================================================================
// RTDEReceive - Safety Status
// ============================================================================

UR_RTDE_API bool ur_rtde_receive_is_protective_stopped(ur_rtde_receive_t* handle);

UR_RTDE_API bool ur_rtde_receive_is_emergency_stopped(ur_rtde_receive_t* handle);

UR_RTDE_API uint32_t ur_rtde_receive_get_robot_status(ur_rtde_receive_t* handle);

UR_RTDE_API uint32_t ur_rtde_receive_get_safety_status_bits(ur_rtde_receive_t* handle);

// ============================================================================
// RTDEReceive - Analog I/O
// ============================================================================

UR_RTDE_API double ur_rtde_receive_get_standard_analog_input(
    ur_rtde_receive_t* handle,
    uint8_t input_id);

UR_RTDE_API double ur_rtde_receive_get_standard_analog_output(
    ur_rtde_receive_t* handle,
    uint8_t output_id);

// ============================================================================
// RTDEControl - Force Mode & Advanced Control
// ============================================================================

UR_RTDE_API ur_rtde_status_t ur_rtde_control_force_mode(
    ur_rtde_control_t* handle,
    const double* task_frame,
    size_t task_frame_size,
    const int* selection_vector,
    size_t selection_vector_size,
    const double* wrench,
    size_t wrench_size,
    int type,
    const double* limits,
    size_t limits_size);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_force_mode_stop(
    ur_rtde_control_t* handle);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_zero_ft_sensor(
    ur_rtde_control_t* handle);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_jog_start(
    ur_rtde_control_t* handle,
    const double* speeds,
    size_t speeds_size,
    int feature);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_jog_stop(
    ur_rtde_control_t* handle);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_teach_mode(
    ur_rtde_control_t* handle);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_end_teach_mode(
    ur_rtde_control_t* handle);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_servo_l(
    ur_rtde_control_t* handle,
    const double* pose,
    size_t pose_size,
    double speed,
    double acceleration,
    double time,
    double lookahead_time,
    double gain);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_trigger_protective_stop(
    ur_rtde_control_t* handle);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_force_mode_set_damping(
    ur_rtde_control_t* handle,
    double damping);

UR_RTDE_API ur_rtde_status_t ur_rtde_control_force_mode_set_gain_scaling(
    ur_rtde_control_t* handle,
    double scaling);

// ============================================================================
// RTDEControl - Custom Script
// ============================================================================

/** Send custom URScript to the controller */
UR_RTDE_API ur_rtde_status_t ur_rtde_control_send_custom_script(
    ur_rtde_control_t* handle,
    const char* script);

// ============================================================================
// RTDEReceive - RTDE Output Registers
// ============================================================================

/** Get RTDE output integer register */
UR_RTDE_API int32_t ur_rtde_receive_get_output_int_register(
    ur_rtde_receive_t* handle,
    uint16_t reg);

/** Get RTDE output double register */
UR_RTDE_API double ur_rtde_receive_get_output_double_register(
    ur_rtde_receive_t* handle,
    uint16_t reg);

// ============================================================================
// RTDEIO Interface
// ============================================================================

typedef struct ur_rtde_io ur_rtde_io_t;

UR_RTDE_API ur_rtde_io_t* ur_rtde_io_create(
    const char* hostname,
    uint16_t flags);

UR_RTDE_API void ur_rtde_io_destroy(ur_rtde_io_t* handle);

UR_RTDE_API bool ur_rtde_io_is_connected(ur_rtde_io_t* handle);

UR_RTDE_API ur_rtde_status_t ur_rtde_io_set_standard_digital_out(
    ur_rtde_io_t* handle,
    uint8_t output_id,
    bool signal_level);

UR_RTDE_API ur_rtde_status_t ur_rtde_io_set_tool_digital_out(
    ur_rtde_io_t* handle,
    uint8_t output_id,
    bool signal_level);

UR_RTDE_API ur_rtde_status_t ur_rtde_io_set_analog_output_voltage(
    ur_rtde_io_t* handle,
    uint8_t output_id,
    double voltage_ratio);

UR_RTDE_API ur_rtde_status_t ur_rtde_io_set_analog_output_current(
    ur_rtde_io_t* handle,
    uint8_t output_id,
    double current_ratio);

UR_RTDE_API ur_rtde_status_t ur_rtde_io_set_speed_slider(
    ur_rtde_io_t* handle,
    double speed);

/** Set RTDE input integer register */
UR_RTDE_API ur_rtde_status_t ur_rtde_io_set_input_int_register(
    ur_rtde_io_t* handle,
    uint16_t reg,
    int32_t value);

/** Set RTDE input double register */
UR_RTDE_API ur_rtde_status_t ur_rtde_io_set_input_double_register(
    ur_rtde_io_t* handle,
    uint16_t reg,
    double value);

UR_RTDE_API void ur_rtde_io_disconnect(ur_rtde_io_t* handle);

// ============================================================================
// Robotiq Gripper Interface (robotiq_gripper.h)
// ============================================================================

/** Create a Robotiq gripper handle (port defaults to 63352) */
UR_RTDE_API ur_rtde_robotiq_gripper_t* ur_rtde_robotiq_gripper_create(
    const char* hostname,
    int port,
    bool verbose);

/** Destroy a Robotiq gripper handle */
UR_RTDE_API void ur_rtde_robotiq_gripper_destroy(
    ur_rtde_robotiq_gripper_t* handle);

/** Connect with timeout (ms) */
UR_RTDE_API ur_rtde_status_t ur_rtde_robotiq_gripper_connect(
    ur_rtde_robotiq_gripper_t* handle,
    uint32_t timeout_ms);

/** Disconnect the gripper */
UR_RTDE_API void ur_rtde_robotiq_gripper_disconnect(
    ur_rtde_robotiq_gripper_t* handle);

/** Check connection */
UR_RTDE_API bool ur_rtde_robotiq_gripper_is_connected(
    ur_rtde_robotiq_gripper_t* handle);

/** Activate the gripper (optionally auto-calibrate) */
UR_RTDE_API ur_rtde_status_t ur_rtde_robotiq_gripper_activate(
    ur_rtde_robotiq_gripper_t* handle,
    bool auto_calibrate);

/** Run auto calibration with optional speed (-1 uses default) */
UR_RTDE_API ur_rtde_status_t ur_rtde_robotiq_gripper_auto_calibrate(
    ur_rtde_robotiq_gripper_t* handle,
    float speed);

/** Check if gripper is active */
UR_RTDE_API bool ur_rtde_robotiq_gripper_is_active(
    ur_rtde_robotiq_gripper_t* handle);

/** Get open/closed/current position */
UR_RTDE_API float ur_rtde_robotiq_gripper_get_open_position(
    ur_rtde_robotiq_gripper_t* handle);

UR_RTDE_API float ur_rtde_robotiq_gripper_get_closed_position(
    ur_rtde_robotiq_gripper_t* handle);

UR_RTDE_API float ur_rtde_robotiq_gripper_get_current_position(
    ur_rtde_robotiq_gripper_t* handle);

UR_RTDE_API bool ur_rtde_robotiq_gripper_is_open(
    ur_rtde_robotiq_gripper_t* handle);

UR_RTDE_API bool ur_rtde_robotiq_gripper_is_closed(
    ur_rtde_robotiq_gripper_t* handle);

/** Move commands */
UR_RTDE_API int ur_rtde_robotiq_gripper_move(
    ur_rtde_robotiq_gripper_t* handle,
    float position,
    float speed,
    float force,
    int move_mode);

UR_RTDE_API int ur_rtde_robotiq_gripper_open(
    ur_rtde_robotiq_gripper_t* handle,
    float speed,
    float force,
    int move_mode);

UR_RTDE_API int ur_rtde_robotiq_gripper_close(
    ur_rtde_robotiq_gripper_t* handle,
    float speed,
    float force,
    int move_mode);

UR_RTDE_API ur_rtde_status_t ur_rtde_robotiq_gripper_emergency_release(
    ur_rtde_robotiq_gripper_t* handle,
    int direction,
    int move_mode);

/** Status and configuration */
UR_RTDE_API int ur_rtde_robotiq_gripper_fault_status(
    ur_rtde_robotiq_gripper_t* handle);

UR_RTDE_API ur_rtde_status_t ur_rtde_robotiq_gripper_set_unit(
    ur_rtde_robotiq_gripper_t* handle,
    int param,
    int unit);

UR_RTDE_API ur_rtde_status_t ur_rtde_robotiq_gripper_set_position_range_mm(
    ur_rtde_robotiq_gripper_t* handle,
    int range_mm);

UR_RTDE_API float ur_rtde_robotiq_gripper_set_speed(
    ur_rtde_robotiq_gripper_t* handle,
    float speed);

UR_RTDE_API float ur_rtde_robotiq_gripper_set_force(
    ur_rtde_robotiq_gripper_t* handle,
    float force);

UR_RTDE_API int ur_rtde_robotiq_gripper_object_detection_status(
    ur_rtde_robotiq_gripper_t* handle);

UR_RTDE_API int ur_rtde_robotiq_gripper_wait_for_motion_complete(
    ur_rtde_robotiq_gripper_t* handle);

/** Raw variable access */
UR_RTDE_API ur_rtde_status_t ur_rtde_robotiq_gripper_set_var(
    ur_rtde_robotiq_gripper_t* handle,
    const char* name,
    int value);

UR_RTDE_API int ur_rtde_robotiq_gripper_get_var(
    ur_rtde_robotiq_gripper_t* handle,
    const char* name);

UR_RTDE_API ur_rtde_status_t ur_rtde_robotiq_gripper_set_vars(
    ur_rtde_robotiq_gripper_t* handle,
    const char** names,
    const int* values,
    size_t count);

UR_RTDE_API ur_rtde_status_t ur_rtde_robotiq_gripper_get_vars(
    ur_rtde_robotiq_gripper_t* handle,
    const char** names,
    size_t count,
    int* values_out);

UR_RTDE_API ur_rtde_status_t ur_rtde_robotiq_gripper_get_native_position_range(
    ur_rtde_robotiq_gripper_t* handle,
    int* min_position,
    int* max_position);

UR_RTDE_API ur_rtde_status_t ur_rtde_robotiq_gripper_set_native_position_range(
    ur_rtde_robotiq_gripper_t* handle,
    int min_position,
    int max_position);

/** Last error string */
UR_RTDE_API const char* ur_rtde_robotiq_gripper_get_last_error(
    ur_rtde_robotiq_gripper_t* handle);

#ifdef __cplusplus
}
#endif

#endif // UR_RTDE_C_API_H
