#ifndef GCS_H
#define GCS_H

#include <condition_variable>
#include <cpplogging/cpplogging.h>
#include <mavlink_cpp/mavlink/common/common.hpp>
#include <mutex>
#include <netinet/in.h>
#include <string>

namespace mavlink_cpp {

using namespace cpplogging;

class GCS : public Logger {

public:
  GCS(uint16_t ownPort);
  void Start();
  void SetManualControl(int16_t x, int16_t y, int16_t z, int16_t r);

private:
#define BUFFER_LENGTH                                                          \
  2041 // minimum buffer size that can be used with qnx (I don't know why)

  int _sockfd;
  struct sockaddr_in _ardupilotAddr;
  struct sockaddr_in _locAddr;
  uint8_t _buf[BUFFER_LENGTH];
  uint8_t _txbuf[BUFFER_LENGTH];
  void _RunRxWork();
  void _RunHeartBeatWork();
  void _RunManualControlWork();

  bool _ardupilotDetected;
  std::mutex _ardupilotAddrMutex;
  std::condition_variable _ardupilotAddrCond;

  std::mutex _manual_control_msg_mutex;
  mavlink::common::msg::MANUAL_CONTROL _manual_control_msg;
};
}
#endif // GCS_H
