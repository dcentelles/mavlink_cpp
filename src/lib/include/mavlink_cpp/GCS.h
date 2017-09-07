#ifndef GCS_H
#define GCS_H

#include <cpplogging/cpplogging.h>
#include <netinet/in.h>
#include <string>
namespace mavlink_cpp {
using namespace cpplogging;
class GCS : public Logger {

public:
  GCS(uint16_t ownPort); //, std::string dstAddr, uint16_t dstPort);
  void Start();

private:
#define BUFFER_LENGTH                                                          \
  2041 // minimum buffer size that can be used with qnx (I don't know why)

  // char _ardupilotIp[100];
  int _sockfd;
  // struct sockaddr_in _arupilotAddr;
  struct sockaddr_in _locAddr;
  uint8_t _buf[BUFFER_LENGTH];
  void _RxWork();
  void _TxWork();
};
}
#endif // GCS_H
