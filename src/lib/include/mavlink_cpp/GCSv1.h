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

class GCSv1 : public Logger {

public:
  GCSv1(uint16_t ownPort);
  void Start();
  void SetManualControl(int16_t x, int16_t y, int16_t z, int16_t r);
  void SetDepthHoldMode();
  void SetStabilizeMode();
  void SetManualMode();
  void EnableGPSMock(bool v);
  void SendGPSOrigin(uint32_t lat, uint32_t lon);
  void Arm(bool);
  void WaitForNEDUpdate();
  mavlink_local_position_ned_t GetNED();
  mavlink_global_position_int_t GetGPS();
  mavlink_scaled_imu2_t GetScaledIMU2();
  mavlink_attitude_quaternion_t GetAttitudeQuaternion();
  mavlink_attitude_t GetAttitude();

  void DebugGlobalPositionInt(mavlink_global_position_int_t &msg);
  void DebugScaledIMU2(mavlink_scaled_imu2_t &msg);
  void DebugRawIMU(mavlink_raw_imu_t &msg);
  void DebugLocalPositonNED(mavlink_local_position_ned_t &msg);
  void DebugAttitudeQuaternion(mavlink_attitude_quaternion_t &msg);
  void DebugAttitude(mavlink_attitude_t &msg);

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
  void _RunGPSMock();

  void _OrderManualMode();

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

  FLY_MODE_R _currentMode;
  bool _enableGPSMock;

  // Received mavlink messages:
  std::mutex _gposint_mutex;
  std::condition_variable _gposint_cond;
  mavlink_global_position_int_t _gposint;
  bool _gposint_updated, _gposint_validOrigin;

  std::mutex _lposned_mutex;
  std::condition_variable _lposned_cond;
  mavlink_local_position_ned_t _lposned;
  bool _lposned_updated;

  std::mutex _scaledImu2_mutex;
  std::condition_variable _scaledImu2_cond;
  mavlink_scaled_imu2_t _scaledImu2;
  bool _scaledImu2_updated;

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
};
}
#endif // GCSV1_H
