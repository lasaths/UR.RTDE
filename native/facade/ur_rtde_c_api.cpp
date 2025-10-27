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

ur_rtde_status_t ur_rtde_control_trigger_watchdog(ur_rtde_control_t* handle) {
  CHECK_HANDLE(handle, UR_RTDE_ERROR_INVALID_HANDLE);
  TRY_CATCH_BLOCK(
    handle->control->triggerWatchdog();
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
    return handle->receive->getStandardDigitalIn(index);
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
    return handle->receive->getStandardDigitalOut(index);
  } catch (...) {
    return false;
  }
}

const char* ur_rtde_receive_get_last_error(ur_rtde_receive_t* handle) {
  if (!handle) return "Invalid handle";
  return handle->last_error.c_str();
}
