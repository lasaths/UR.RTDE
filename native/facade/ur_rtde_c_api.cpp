// ur_rtde_c_api.cpp
// Implementation of C ABI façade over ur_rtde C++
// Handles exception boundaries and state management

#include "ur_rtde_c_api.h"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <memory>
#include <string>
#include <cstring>
#include <exception>

using namespace ur_rtde;

// ============================================================================
// Opaque Handle Structures
// ============================================================================

struct ur_rtde_control_handle {
  std::unique_ptr<RTDEControlInterface> control;
  std::string last_error;
  
  ur_rtde_control_handle(const std::string& hostname, double freq, uint16_t flags, int port)
    : control(std::make_unique<RTDEControlInterface>(hostname, freq, flags, port)) {}
};

struct ur_rtde_receive_handle {
  std::unique_ptr<RTDEReceiveInterface> receive;
  std::string last_error;
  
  ur_rtde_receive_handle(const std::string& hostname, double freq)
    : receive(std::make_unique<RTDEReceiveInterface>(hostname, freq)) {}
};

// ============================================================================
// Helper Macros
// ============================================================================

#define CHECK_HANDLE(handle, error_code) \
  if (!handle) return error_code;

#define TRY_CATCH_BLOCK(code, handle, error_code) \
  try { \
    code \
  } catch (const std::exception& e) { \
    if (handle) handle->last_error = e.what(); \
    return error_code; \
  } catch (...) { \
    if (handle) handle->last_error = "Unknown exception"; \
    return error_code; \
  }

// ============================================================================
// Control Interface Implementation
// ============================================================================

ur_rtde_control_t* ur_rtde_control_create(
  const char* hostname,
  double frequency,
  uint16_t flags,
  int ur_cap_port
) {
  if (!hostname) return nullptr;
  
  try {
    auto handle = new ur_rtde_control_handle(hostname, frequency, flags, ur_cap_port);
    if (!handle->control->isConnected()) {
      delete handle;
      return nullptr;
    }
    return handle;
  } catch (...) {
    return nullptr;
  }
}

void ur_rtde_control_destroy(ur_rtde_control_t* handle) {
  if (handle) {
    try {
      handle->control->disconnect();
    } catch (...) {}
    delete handle;
  }
}

bool ur_rtde_control_is_connected(ur_rtde_control_t* handle) {
  if (!handle) return false;
  try {
    return handle->control->isConnected();
  } catch (...) {
    return false;
  }
}

ur_rtde_status_t ur_rtde_control_disconnect(ur_rtde_control_t* handle) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  TRY_CATCH_BLOCK(
    handle->control->disconnect();
    return UR_RTDE_OK;
  , handle, UR_RTDE_ERROR_CONNECTION);
}

ur_rtde_status_t ur_rtde_control_reconnect(ur_rtde_control_t* handle) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  TRY_CATCH_BLOCK(
    bool success = handle->control->reconnect();
    return success ? UR_RTDE_OK : UR_RTDE_ERROR_CONNECTION;
  , handle, UR_RTDE_ERROR_CONNECTION);
}

ur_rtde_status_t ur_rtde_control_move_j(
  ur_rtde_control_t* handle,
  const double q[6],
  double speed,
  double acceleration,
  bool asynchronous
) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  if (!q) return UR_RTDE_ERROR_INVALID_PARAM;
  
  TRY_CATCH_BLOCK(
    std::vector<double> q_vec(q, q + 6);
    bool success = handle->control->moveJ(q_vec, speed, acceleration, asynchronous);
    return success ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
  , handle, UR_RTDE_ERROR_COMMAND_FAILED);
}

ur_rtde_status_t ur_rtde_control_move_l(
  ur_rtde_control_t* handle,
  const double pose[6],
  double speed,
  double acceleration,
  bool asynchronous
) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  if (!pose) return UR_RTDE_ERROR_INVALID_PARAM;
  
  TRY_CATCH_BLOCK(
    std::vector<double> pose_vec(pose, pose + 6);
    bool success = handle->control->moveL(pose_vec, speed, acceleration, asynchronous);
    return success ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
  , handle, UR_RTDE_ERROR_COMMAND_FAILED);
}

ur_rtde_status_t ur_rtde_control_stop_j(
  ur_rtde_control_t* handle,
  double acceleration,
  bool asynchronous
) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  TRY_CATCH_BLOCK(
    handle->control->stopJ(acceleration, asynchronous);
    return UR_RTDE_OK;
  , handle, UR_RTDE_ERROR_COMMAND_FAILED);
}

ur_rtde_status_t ur_rtde_control_stop_l(
  ur_rtde_control_t* handle,
  double acceleration,
  bool asynchronous
) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  TRY_CATCH_BLOCK(
    handle->control->stopL(acceleration, asynchronous);
    return UR_RTDE_OK;
  , handle, UR_RTDE_ERROR_COMMAND_FAILED);
}

ur_rtde_status_t ur_rtde_control_speed_j(
  ur_rtde_control_t* handle,
  const double qd[6],
  double acceleration,
  double time
) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  if (!qd) return UR_RTDE_ERROR_INVALID_PARAM;
  
  TRY_CATCH_BLOCK(
    std::vector<double> qd_vec(qd, qd + 6);
    bool success = handle->control->speedJ(qd_vec, acceleration, time);
    return success ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
  , handle, UR_RTDE_ERROR_COMMAND_FAILED);
}

ur_rtde_status_t ur_rtde_control_speed_l(
  ur_rtde_control_t* handle,
  const double xd[6],
  double acceleration,
  double time
) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  if (!xd) return UR_RTDE_ERROR_INVALID_PARAM;
  
  TRY_CATCH_BLOCK(
    std::vector<double> xd_vec(xd, xd + 6);
    bool success = handle->control->speedL(xd_vec, acceleration, time);
    return success ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
  , handle, UR_RTDE_ERROR_COMMAND_FAILED);
}

ur_rtde_status_t ur_rtde_control_servo_j(
  ur_rtde_control_t* handle,
  const double q[6],
  double speed,
  double acceleration,
  double time,
  double lookahead_time,
  double gain
) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  if (!q) return UR_RTDE_ERROR_INVALID_PARAM;
  
  TRY_CATCH_BLOCK(
    std::vector<double> q_vec(q, q + 6);
    bool success = handle->control->servoJ(q_vec, speed, acceleration, time, lookahead_time, gain);
    return success ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
  , handle, UR_RTDE_ERROR_COMMAND_FAILED);
}

ur_rtde_status_t ur_rtde_control_set_tcp(
  ur_rtde_control_t* handle,
  const double tcp_pose[6]
) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  if (!tcp_pose) return UR_RTDE_ERROR_INVALID_PARAM;
  
  TRY_CATCH_BLOCK(
    std::vector<double> tcp_vec(tcp_pose, tcp_pose + 6);
    bool success = handle->control->setTcp(tcp_vec);
    return success ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
  , handle, UR_RTDE_ERROR_COMMAND_FAILED);
}

ur_rtde_status_t ur_rtde_control_set_payload(
  ur_rtde_control_t* handle,
  double mass,
  const double cog[3]
) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  if (!cog) return UR_RTDE_ERROR_INVALID_PARAM;
  
  TRY_CATCH_BLOCK(
    std::vector<double> cog_vec(cog, cog + 3);
    bool success = handle->control->setPayload(mass, cog_vec);
    return success ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
  , handle, UR_RTDE_ERROR_COMMAND_FAILED);
}

ur_rtde_status_t ur_rtde_control_kick_watchdog(ur_rtde_control_t* handle) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  TRY_CATCH_BLOCK(
    handle->control->kickWatchdog();
    return UR_RTDE_OK;
  , handle, UR_RTDE_ERROR_COMMAND_FAILED);
}

const char* ur_rtde_control_get_last_error(ur_rtde_control_t* handle) {
  if (!handle) return "Invalid handle";
  return handle->last_error.c_str();
}

// ============================================================================
// Receive Interface Implementation
// ============================================================================

ur_rtde_receive_t* ur_rtde_receive_create(
  const char* hostname,
  double frequency,
  uint16_t flags
) {
  if (!hostname) return nullptr;
  
  try {
    auto handle = new ur_rtde_receive_handle(hostname, frequency);
    if (!handle->receive->isConnected()) {
      delete handle;
      return nullptr;
    }
    return handle;
  } catch (...) {
    return nullptr;
  }
}

void ur_rtde_receive_destroy(ur_rtde_receive_t* handle) {
  if (handle) {
    try {
      handle->receive->disconnect();
    } catch (...) {}
    delete handle;
  }
}

bool ur_rtde_receive_is_connected(ur_rtde_receive_t* handle) {
  if (!handle) return false;
  try {
    return handle->receive->isConnected();
  } catch (...) {
    return false;
  }
}

ur_rtde_status_t ur_rtde_receive_get_actual_q(
  ur_rtde_receive_t* handle,
  double q_out[6]
) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  if (!q_out) return UR_RTDE_ERROR_INVALID_PARAM;
  
  TRY_CATCH_BLOCK(
    std::vector<double> q = handle->receive->getActualQ();
    if (q.size() != 6) return UR_RTDE_ERROR_UNKNOWN;
    std::copy(q.begin(), q.end(), q_out);
    return UR_RTDE_OK;
  , handle, UR_RTDE_ERROR_UNKNOWN);
}

ur_rtde_status_t ur_rtde_receive_get_actual_tcp_pose(
  ur_rtde_receive_t* handle,
  double pose_out[6]
) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  if (!pose_out) return UR_RTDE_ERROR_INVALID_PARAM;
  
  TRY_CATCH_BLOCK(
    std::vector<double> pose = handle->receive->getActualTCPPose();
    if (pose.size() != 6) return UR_RTDE_ERROR_UNKNOWN;
    std::copy(pose.begin(), pose.end(), pose_out);
    return UR_RTDE_OK;
  , handle, UR_RTDE_ERROR_UNKNOWN);
}

ur_rtde_status_t ur_rtde_receive_get_actual_qd(
  ur_rtde_receive_t* handle,
  double qd_out[6]
) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  if (!qd_out) return UR_RTDE_ERROR_INVALID_PARAM;
  
  TRY_CATCH_BLOCK(
    std::vector<double> qd = handle->receive->getActualQd();
    if (qd.size() != 6) return UR_RTDE_ERROR_UNKNOWN;
    std::copy(qd.begin(), qd.end(), qd_out);
    return UR_RTDE_OK;
  , handle, UR_RTDE_ERROR_UNKNOWN);
}

ur_rtde_status_t ur_rtde_receive_get_actual_tcp_speed(
  ur_rtde_receive_t* handle,
  double speed_out[6]
) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  if (!speed_out) return UR_RTDE_ERROR_INVALID_PARAM;
  
  TRY_CATCH_BLOCK(
    std::vector<double> speed = handle->receive->getActualTCPSpeed();
    if (speed.size() != 6) return UR_RTDE_ERROR_UNKNOWN;
    std::copy(speed.begin(), speed.end(), speed_out);
    return UR_RTDE_OK;
  , handle, UR_RTDE_ERROR_UNKNOWN);
}

int32_t ur_rtde_receive_get_robot_mode(ur_rtde_receive_t* handle) {
  if (!handle) return -1;
  try {
    return handle->receive->getRobotMode();
  } catch (...) {
    return -1;
  }
}

int32_t ur_rtde_receive_get_safety_mode(ur_rtde_receive_t* handle) {
  if (!handle) return -1;
  try {
    return handle->receive->getSafetyMode();
  } catch (...) {
    return -1;
  }
}

int32_t ur_rtde_receive_get_runtime_state(ur_rtde_receive_t* handle) {
  if (!handle) return -1;
  try {
    return handle->receive->getRuntimeState();
  } catch (...) {
    return -1;
  }
}

bool ur_rtde_receive_get_standard_digital_in(
  ur_rtde_receive_t* handle,
  int index
) {
  if (!handle || index < 0 || index > 7) return false;
  try {
    return handle->receive->getDigitalInState(index);
  } catch (...) {
    return false;
  }
}

bool ur_rtde_receive_get_standard_digital_out(
  ur_rtde_receive_t* handle,
  int index
) {
  if (!handle || index < 0 || index > 7) return false;
  try {
    return handle->receive->getDigitalOutState(index);
  } catch (...) {
    return false;
  }
}

const char* ur_rtde_receive_get_last_error(ur_rtde_receive_t* handle) {
  if (!handle) return "Invalid handle";
  return handle->last_error.c_str();
}

// ============================================================================
// RTDEControl - Kinematics
// ============================================================================

ur_rtde_status_t ur_rtde_control_get_inverse_kinematics(
    ur_rtde_control_t* handle,
    const double* x,
    size_t x_size,
    double* q_out,
    size_t q_size)
{
    if (!handle || !x || !q_out || x_size != 6 || q_size != 6) {
        return UR_RTDE_ERROR_INVALID_PARAM;
    }
    
    try {
        std::vector<double> x_vec(x, x + x_size);
        std::vector<double> q = handle->control->getInverseKinematics(x_vec);
        
        if (q.size() != 6) {
            return UR_RTDE_ERROR_COMMAND_FAILED;
        }
        
        std::copy(q.begin(), q.end(), q_out);
        return UR_RTDE_OK;
    } catch (const std::exception& e) {
        std::cerr << "getInverseKinematics: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_get_forward_kinematics(
    ur_rtde_control_t* handle,
    const double* q,
    size_t q_size,
    double* x_out,
    size_t x_size)
{
    if (!handle || !q || !x_out || q_size != 6 || x_size != 6) {
        return UR_RTDE_ERROR_INVALID_PARAM;
    }
    
    try {
        std::vector<double> q_vec(q, q + q_size);
        std::vector<double> x = handle->control->getForwardKinematics(q_vec);
        
        if (x.size() != 6) {
            return UR_RTDE_ERROR_COMMAND_FAILED;
        }
        
        std::copy(x.begin(), x.end(), x_out);
        return UR_RTDE_OK;
    } catch (const std::exception& e) {
        std::cerr << "getForwardKinematics: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

bool ur_rtde_control_get_inverse_kinematics_has_solution(
    ur_rtde_control_t* handle,
    const double* x,
    size_t x_size)
{
    if (!handle || !x || x_size != 6) {
        return false;
    }
    
    try {
        std::vector<double> x_vec(x, x + x_size);
        return handle->control->getInverseKinematicsHasSolution(x_vec);
    } catch (...) {
        return false;
    }
}

// ============================================================================
// RTDEControl - Additional Movement
// ============================================================================

ur_rtde_status_t ur_rtde_control_servo_c(
    ur_rtde_control_t* handle,
    const double* pose,
    size_t pose_size,
    double speed,
    double acceleration,
    double blend)
{
    if (!handle || !pose || pose_size != 6) {
        return UR_RTDE_ERROR_INVALID_PARAM;
    }
    
    try {
        std::vector<double> pose_vec(pose, pose + pose_size);
        bool result = handle->control->servoC(pose_vec, speed, acceleration, blend);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        std::cerr << "servoC: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_servo_stop(
    ur_rtde_control_t* handle,
    double acceleration)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }
    
    try {
        bool result = handle->control->servoStop(acceleration);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        std::cerr << "servoStop: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_speed_stop(
    ur_rtde_control_t* handle,
    double acceleration)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }
    
    try {
        bool result = handle->control->speedStop(acceleration);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        std::cerr << "speedStop: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

// ============================================================================
// RTDEControl - Safety & Status
// ============================================================================

bool ur_rtde_control_is_program_running(ur_rtde_control_t* handle)
{
    if (!handle) return false;
    try {
        return handle->control->isProgramRunning();
    } catch (...) {
        return false;
    }
}

bool ur_rtde_control_is_steady(ur_rtde_control_t* handle)
{
    if (!handle) return false;
    try {
        return handle->control->isSteady();
    } catch (...) {
        return false;
    }
}

uint32_t ur_rtde_control_get_robot_status(ur_rtde_control_t* handle)
{
    if (!handle) return 0;
    try {
        return handle->control->getRobotStatus();
    } catch (...) {
        return 0;
    }
}

// ============================================================================
// RTDEControl - RTDE Register & Custom Script
// ============================================================================

ur_rtde_status_t ur_rtde_control_set_input_int_register(
    ur_rtde_control_t* handle,
    uint16_t reg,
    int32_t value)
{
    if (!handle) return UR_RTDE_ERROR_INVALID_HANDLE;
    try {
        bool result = handle->control->setInputIntRegister(reg, value);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_set_input_double_register(
    ur_rtde_control_t* handle,
    uint16_t reg,
    double value)
{
    if (!handle) return UR_RTDE_ERROR_INVALID_HANDLE;
    try {
        bool result = handle->control->setInputDoubleRegister(reg, value);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_set_input_bit_register(
    ur_rtde_control_t* handle,
    uint16_t reg,
    bool value)
{
    if (!handle) return UR_RTDE_ERROR_INVALID_HANDLE;
    try {
        bool result = handle->control->setInputBitRegister(reg, value);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_send_custom_script(
    ur_rtde_control_t* handle,
    const char* script)
{
    if (!handle) return UR_RTDE_ERROR_INVALID_HANDLE;
    if (!script) return UR_RTDE_ERROR_INVALID_PARAM;
    try {
        bool result = handle->control->sendCustomScript(std::string(script));
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

// ============================================================================
// RTDEReceive - Extended Data
// ============================================================================

ur_rtde_status_t ur_rtde_receive_get_target_q(
    ur_rtde_receive_t* handle,
    double* q_out,
    size_t q_size)
{
    if (!handle || !q_out || q_size != 6) {
        return UR_RTDE_ERROR_INVALID_PARAM;
    }
    
    try {
        std::vector<double> q = handle->receive->getTargetQ();
        if (q.size() != 6) {
            return UR_RTDE_ERROR_COMMAND_FAILED;
        }
        std::copy(q.begin(), q.end(), q_out);
        return UR_RTDE_OK;
    } catch (const std::exception& e) {
        std::cerr << "getTargetQ: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_receive_get_target_tcp_pose(
    ur_rtde_receive_t* handle,
    double* pose_out,
    size_t pose_size)
{
    if (!handle || !pose_out || pose_size != 6) {
        return UR_RTDE_ERROR_INVALID_PARAM;
    }
    
    try {
        std::vector<double> pose = handle->receive->getTargetTCPPose();
        if (pose.size() != 6) {
            return UR_RTDE_ERROR_COMMAND_FAILED;
        }
        std::copy(pose.begin(), pose.end(), pose_out);
        return UR_RTDE_OK;
    } catch (const std::exception& e) {
        std::cerr << "getTargetTCPPose: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_receive_get_actual_tcp_force(
    ur_rtde_receive_t* handle,
    double* force_out,
    size_t force_size)
{
    if (!handle || !force_out || force_size != 6) {
        return UR_RTDE_ERROR_INVALID_PARAM;
    }
    
    try {
        std::vector<double> force = handle->receive->getActualTCPForce();
        if (force.size() != 6) {
            return UR_RTDE_ERROR_COMMAND_FAILED;
        }
        std::copy(force.begin(), force.end(), force_out);
        return UR_RTDE_OK;
    } catch (const std::exception& e) {
        std::cerr << "getActualTCPForce: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_receive_get_joint_temperatures(
    ur_rtde_receive_t* handle,
    double* temps_out,
    size_t temps_size)
{
    if (!handle || !temps_out || temps_size != 6) {
        return UR_RTDE_ERROR_INVALID_PARAM;
    }
    
    try {
        std::vector<double> temps = handle->receive->getJointTemperatures();
        if (temps.size() != 6) {
            return UR_RTDE_ERROR_COMMAND_FAILED;
        }
        std::copy(temps.begin(), temps.end(), temps_out);
        return UR_RTDE_OK;
    } catch (const std::exception& e) {
        std::cerr << "getJointTemperatures: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_receive_get_actual_current(
    ur_rtde_receive_t* handle,
    double* current_out,
    size_t current_size)
{
    if (!handle || !current_out || current_size != 6) {
        return UR_RTDE_ERROR_INVALID_PARAM;
    }
    
    try {
        std::vector<double> current = handle->receive->getActualCurrent();
        if (current.size() != 6) {
            return UR_RTDE_ERROR_COMMAND_FAILED;
        }
        std::copy(current.begin(), current.end(), current_out);
        return UR_RTDE_OK;
    } catch (const std::exception& e) {
        std::cerr << "getActualCurrent: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

// ============================================================================
// RTDEReceive - Safety Status
// ============================================================================

bool ur_rtde_receive_is_protective_stopped(ur_rtde_receive_t* handle)
{
    if (!handle) return false;
    try {
        return handle->receive->isProtectiveStopped();
    } catch (...) {
        return false;
    }
}

bool ur_rtde_receive_is_emergency_stopped(ur_rtde_receive_t* handle)
{
    if (!handle) return false;
    try {
        return handle->receive->isEmergencyStopped();
    } catch (...) {
        return false;
    }
}

uint32_t ur_rtde_receive_get_robot_status(ur_rtde_receive_t* handle)
{
    if (!handle) return 0;
    try {
        return handle->receive->getRobotStatus();
    } catch (...) {
        return 0;
    }
}

uint32_t ur_rtde_receive_get_safety_status_bits(ur_rtde_receive_t* handle)
{
    if (!handle) return 0;
    try {
        return handle->receive->getSafetyStatusBits();
    } catch (...) {
        return 0;
    }
}

// ============================================================================
// RTDEReceive - RTDE Output Registers
// ============================================================================

int32_t ur_rtde_receive_get_output_int_register(
    ur_rtde_receive_t* handle,
    uint16_t reg)
{
    if (!handle) return 0;
    try {
        return handle->receive->getOutputIntRegister(reg);
    } catch (...) {
        return 0;
    }
}

double ur_rtde_receive_get_output_double_register(
    ur_rtde_receive_t* handle,
    uint16_t reg)
{
    if (!handle) return 0.0;
    try {
        return handle->receive->getOutputDoubleRegister(reg);
    } catch (...) {
        return 0.0;
    }
}

bool ur_rtde_receive_get_output_bit_register(
    ur_rtde_receive_t* handle,
    uint16_t reg)
{
    if (!handle) return false;
    try {
        return handle->receive->getOutputBitRegister(reg);
    } catch (...) {
        return false;
    }
}

// ============================================================================
// RTDEReceive - Analog I/O
// ============================================================================

double ur_rtde_receive_get_standard_analog_input(
    ur_rtde_receive_t* handle,
    uint8_t input_id)
{
    if (!handle) return 0.0;
    try {
        if (input_id == 0) {
            return handle->receive->getStandardAnalogInput0();
        } else if (input_id == 1) {
            return handle->receive->getStandardAnalogInput1();
        }
        return 0.0;
    } catch (...) {
        return 0.0;
    }
}

double ur_rtde_receive_get_standard_analog_output(
    ur_rtde_receive_t* handle,
    uint8_t output_id)
{
    if (!handle) return 0.0;
    try {
        if (output_id == 0) {
            return handle->receive->getStandardAnalogOutput0();
        } else if (output_id == 1) {
            return handle->receive->getStandardAnalogOutput1();
        }
        return 0.0;
    } catch (...) {
        return 0.0;
    }
}

// ============================================================================
// RTDEControl - Force Mode & Advanced Control
// ============================================================================

ur_rtde_status_t ur_rtde_control_force_mode(
    ur_rtde_control_t* handle,
    const double* task_frame,
    size_t task_frame_size,
    const int* selection_vector,
    size_t selection_vector_size,
    const double* wrench,
    size_t wrench_size,
    int type,
    const double* limits,
    size_t limits_size)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }

    if (task_frame_size != 6 || selection_vector_size != 6 || wrench_size != 6 || limits_size != 6) {
        return UR_RTDE_ERROR_INVALID_PARAM;
    }

    try {
        std::vector<double> task_frame_vec(task_frame, task_frame + task_frame_size);
        std::vector<int> selection_vec(selection_vector, selection_vector + selection_vector_size);
        std::vector<double> wrench_vec(wrench, wrench + wrench_size);
        std::vector<double> limits_vec(limits, limits + limits_size);
        
        bool result = handle->control->forceMode(task_frame_vec, selection_vec, wrench_vec, type, limits_vec);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_force_mode_stop(ur_rtde_control_t* handle)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }

    try {
        bool result = handle->control->forceModeStop();
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_zero_ft_sensor(ur_rtde_control_t* handle)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }

    try {
        bool result = handle->control->zeroFtSensor();
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_jog_start(
    ur_rtde_control_t* handle,
    const double* speeds,
    size_t speeds_size,
    int feature)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }

    if (speeds_size != 6) {
        return UR_RTDE_ERROR_INVALID_PARAM;
    }

    try {
        std::vector<double> speeds_vec(speeds, speeds + speeds_size);
        bool result = handle->control->jogStart(speeds_vec, feature);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_jog_stop(ur_rtde_control_t* handle)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }

    try {
        bool result = handle->control->jogStop();
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_teach_mode(ur_rtde_control_t* handle)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }

    try {
        bool result = handle->control->teachMode();
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_end_teach_mode(ur_rtde_control_t* handle)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }

    try {
        bool result = handle->control->endTeachMode();
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_servo_l(
    ur_rtde_control_t* handle,
    const double* pose,
    size_t pose_size,
    double speed,
    double acceleration,
    double time,
    double lookahead_time,
    double gain)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }

    if (pose_size != 6) {
        return UR_RTDE_ERROR_INVALID_PARAM;
    }

    try {
        std::vector<double> pose_vec(pose, pose + pose_size);
        bool result = handle->control->servoL(pose_vec, speed, acceleration, time, lookahead_time, gain);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_trigger_protective_stop(ur_rtde_control_t* handle)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }

    try {
        bool result = handle->control->triggerProtectiveStop();
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_force_mode_set_damping(
    ur_rtde_control_t* handle,
    double damping)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }

    try {
        bool result = handle->control->forceModeSetDamping(damping);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_control_force_mode_set_gain_scaling(
    ur_rtde_control_t* handle,
    double scaling)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }

    try {
        bool result = handle->control->forceModeSetGainScaling(scaling);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        handle->last_error = e.what();
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

// ============================================================================
// RTDEIO Interface
// ============================================================================

#include <ur_rtde/rtde_io_interface.h>

struct ur_rtde_io {
    std::unique_ptr<ur_rtde::RTDEIOInterface> io;
};

ur_rtde_io_t* ur_rtde_io_create(
    const char* hostname,
    uint16_t flags)
{
    if (!hostname) return nullptr;
    
    try {
        auto handle = new ur_rtde_io();
        bool verbose = (flags & UR_RTDE_FLAG_VERBOSE) != 0;
        handle->io = std::make_unique<ur_rtde::RTDEIOInterface>(hostname, verbose);
        return handle;
    } catch (const std::exception& e) {
        std::cerr << "RTDEIO create failed: " << e.what() << std::endl;
        return nullptr;
    }
}

void ur_rtde_io_destroy(ur_rtde_io_t* handle)
{
    delete handle;
}

bool ur_rtde_io_is_connected(ur_rtde_io_t* handle)
{
    return handle && handle->io;
}

ur_rtde_status_t ur_rtde_io_set_standard_digital_out(
    ur_rtde_io_t* handle,
    uint8_t output_id,
    bool signal_level)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }
    
    try {
        bool result = handle->io->setStandardDigitalOut(output_id, signal_level);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        std::cerr << "setStandardDigitalOut: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_io_set_tool_digital_out(
    ur_rtde_io_t* handle,
    uint8_t output_id,
    bool signal_level)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }
    
    try {
        bool result = handle->io->setToolDigitalOut(output_id, signal_level);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        std::cerr << "setToolDigitalOut: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_io_set_analog_output_voltage(
    ur_rtde_io_t* handle,
    uint8_t output_id,
    double voltage_ratio)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }
    
    try {
        bool result = handle->io->setAnalogOutputVoltage(output_id, voltage_ratio);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        std::cerr << "setAnalogOutputVoltage: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_io_set_analog_output_current(
    ur_rtde_io_t* handle,
    uint8_t output_id,
    double current_ratio)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }
    
    try {
        bool result = handle->io->setAnalogOutputCurrent(output_id, current_ratio);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        std::cerr << "setAnalogOutputCurrent: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

ur_rtde_status_t ur_rtde_io_set_speed_slider(
    ur_rtde_io_t* handle,
    double speed)
{
    if (!handle) {
        return UR_RTDE_ERROR_INVALID_HANDLE;
    }
    
    try {
        bool result = handle->io->setSpeedSlider(speed);
        return result ? UR_RTDE_OK : UR_RTDE_ERROR_COMMAND_FAILED;
    } catch (const std::exception& e) {
        std::cerr << "setSpeedSlider: " << e.what() << std::endl;
        return UR_RTDE_ERROR_COMMAND_FAILED;
    }
}

void ur_rtde_io_disconnect(ur_rtde_io_t* handle)
{
    if (handle && handle->io) {
        handle->io->disconnect();
    }
}
