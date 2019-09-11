#ifndef GCSV1_H
#define GCSV1_H

#include <condition_variable>
#include <cpplogging/cpplogging.h>
#include <mavlink_cpp/mavlink/c_library_v1/common/mavlink.h>
#include <mavlink_cpp/mavlink_types.h>
#include <mutex>
#include <netinet/in.h>
#include <string>

namespace mavlink_cpp {

using namespace cpplogging;

class GCS;
typedef std::shared_ptr<GCS> GCSPtr;

class GCS : public Logger {

public:
  GCS(const uint16_t &ownPort);
  static GCSPtr Create(const uint16_t &ownPort) {
    return GCSPtr(new GCS(ownPort));
  }
  void Start();
  void SetManualControl(int16_t x, int16_t y, int16_t z, int16_t r);
  void SetDepthHoldMode();
  void SetStabilizeMode();
  void SetFlyMode(FLY_MODE_R flymode);
  void SetManualMode();
  void EnableGPSMock(bool v);
  void SendGPSOrigin(uint32_t lat, uint32_t lon);
  void Arm(bool);
  void WaitForNEDUpdate();
  void SendVisionPositionEstimate(mavlink_vision_position_estimate_t &msg);
  void SendGPSFix(mavlink_hil_gps_t &msg);
  void SendGPSInput(mavlink_gps_input_t &msg);
  void
  SendSetPositionTargetLocalNED(mavlink_set_position_target_local_ned_t &msg);

  mavlink_local_position_ned_t GetNED();
  mavlink_global_position_int_t GetGPS();
  mavlink_scaled_imu2_t GetScaledIMU2();
  mavlink_attitude_quaternion_t GetAttitudeQuaternion();
  mavlink_attitude_t GetAttitude();
  mavlink_vfr_hud_t GetVfrHud();

  void DebugGlobalPositionInt(mavlink_global_position_int_t &msg);
  void DebugScaledIMU2(mavlink_scaled_imu2_t &msg);
  void DebugRawIMU(mavlink_raw_imu_t &msg);
  void DebugLocalPositonNED(mavlink_local_position_ned_t &msg);
  void DebugAttitudeQuaternion(mavlink_attitude_quaternion_t &msg);
  void DebugAttitude(mavlink_attitude_t &msg);

  void SetHeartbeatCb(std::function<void(const mavlink_heartbeat_t &)> handler);
  void SetAttitudeCb(std::function<void(const mavlink_attitude_t &)> handler);
  void SetVfrHudCb(std::function<void(const mavlink_vfr_hud_t &)> handler);
  void SetLocalPositionNEDCb(
      std::function<void(const mavlink_local_position_ned_t &)> handler);
  void SetGlobalPositionInt(
      std::function<void(const mavlink_global_position_int_t &)> handler);
  void
  SetSclaedIMU2(std::function<void(const mavlink_scaled_imu2_t &)> handler);
  void SetHomeUpdatedCb(
      std::function<void(const mavlink_home_position_t &)> handler);

  void EnableManualControl(bool enable);
  bool HomeSet() { return _home_position_set; }
  void SetHome(double lat, double lon, double alt);
  bool Armed();

  FLY_MODE_R GetCurrentNavMode();

private:
#define GCS_BUFFER_LENGTH                                                      \
  2041 // minimum buffer size that can be used with qnx (I don't know why)
  bool _armed;
  int _sockfd;
  struct sockaddr_in _ardupilotAddr;
  struct sockaddr_in _locAddr;

  void _RunRxWork();
  void _RunHeartBeatWork();
  void _RunManualControlWork();
  void _RunNavModeWork();
  void _RunGPSMock();

  void _CheckFlyMode(FLY_MODE_R flymode);

  bool _ardupilotDetected;
  std::mutex _ardupilotAddrMutex;
  std::condition_variable _ardupilotAddrCond;

  std::mutex _manual_control_msg_mutex, _socket_mutex;
  mavlink_manual_control_t _manual_control_msg;
  mavlink_gps_input_t _gps_input_msg;

  static const int MODE_BUTTONS_MASK = 15;
  static const int ARM_BUTTON = 64, DISARM_BUTTON = 16, STABILIZE_BUTTON = 2,
                   DEPTH_HOLD_BUTTON = 8;

  int _cmdLongSeq = 0;

  FLY_MODE_R _currentMode, _desiredMode;
  bool _enableGPSMock;
  std::mutex _enableGPSMock_mutex;
  std::condition_variable _enableGPSMock_cond;

  bool _enableManualControl;
  std::mutex _enableManualControl_mutex;
  std::condition_variable _enableManualControl_cond;

  // Received mavlink messages:
  std::mutex _gposint_mutex;
  std::condition_variable _gposint_cond;
  mavlink_global_position_int_t _gposint;
  bool _gposint_updated, _gposint_validOrigin;
  std::function<void(const mavlink_global_position_int_t &)> _gposint_cb =
      [](const mavlink_global_position_int_t &) {};

  std::mutex _lposned_mutex;
  std::condition_variable _lposned_cond;
  mavlink_local_position_ned_t _lposned;
  bool _lposned_updated;
  std::function<void(const mavlink_local_position_ned_t &)> _lposned_cb =
      [](const mavlink_local_position_ned_t &) {};

  std::mutex _scaledImu2_mutex;
  std::condition_variable _scaledImu2_cond;
  mavlink_scaled_imu2_t _scaledImu2;
  bool _scaledImu2_updated;
  std::function<void(const mavlink_scaled_imu2_t &)> _scaledImu2_cb =
      [](const mavlink_scaled_imu2_t &) {};

  std::mutex _rawImu_mutex;
  std::condition_variable _rawImu_cond;
  mavlink_raw_imu_t _rawImu;
  bool _rawImu_updated;

  std::mutex _attitudeQuaternion_mutex;
  std::condition_variable _attitudeQuaternion_cond;
  mavlink_attitude_quaternion_t _attitudeQuaternion;
  bool _attitudeQuaternion_updated;

  std::mutex _attitude_mutex;
  std::condition_variable _attitude_cond;
  mavlink_attitude_t _attitude;
  bool _attitude_updated;
  std::function<void(const mavlink_attitude_t &)> _attitude_cb =
      [](const mavlink_attitude_t &) {};

  std::mutex _vfr_hud_mutex;
  std::condition_variable _vfr_hud_cond;
  mavlink_vfr_hud_t _vfr_hud;
  bool _vfr_hud_updated;
  std::function<void(const mavlink_vfr_hud_t &)> _vfr_hud_cb =
      [](const mavlink_vfr_hud_t &) {};

  std::mutex _hb_mutex;
  std::condition_variable _hb_cond;
  mavlink_heartbeat_t _hb;
  bool _hb_updated;
  std::function<void(const mavlink_heartbeat_t &)> _hb_cb =
      [](const mavlink_heartbeat_t &) {};

  std::mutex _home_mutex;
  std::condition_variable _home_cond;
  mavlink_home_position_t _home;
  bool _home_updated;
  std::function<void(const mavlink_home_position_t &)> _home_cb =
      [](const mavlink_home_position_t &) {};

  bool _home_position_set = false;
};
} // namespace mavlink_cpp
#endif // GCSV1_H
