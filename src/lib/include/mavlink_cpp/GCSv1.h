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
  void Arm(bool);

private:
#define GCS_BUFFER_LENGTH                                                          \
  2041 // minimum buffer size that can be used with qnx (I don't know why)
  bool _armed;
  int _sockfd;
  struct sockaddr_in _ardupilotAddr;
  struct sockaddr_in _locAddr;

  void _RunRxWork();
  void _RunHeartBeatWork();
  void _RunManualControlWork();

  void _OrderManualMode();

  bool _ardupilotDetected;
  std::mutex _ardupilotAddrMutex;
  std::condition_variable _ardupilotAddrCond;

  std::mutex _manual_control_msg_mutex, _socket_mutex;
  mavlink_manual_control_t _manual_control_msg;

  static const int MODE_BUTTONS_MASK = 15;
  static const int ARM_BUTTON = 64, DISARM_BUTTON = 16, STABILIZE_BUTTON = 2,
                   DEPTH_HOLD_BUTTON = 8;

  int _cmdLongSeq = 0;

  FLY_MODE_R _currentMode;
};
}
#endif // GCSV1_H
