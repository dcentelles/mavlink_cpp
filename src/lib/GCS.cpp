#include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
#include <mavlink_cpp/GCS.h>
#include <mavlink_cpp/mavlink/common/common.hpp>
#include <netinet/in.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

namespace mavlink_cpp {

using namespace mavlink;
using namespace mavlink::common;

GCS::GCS(uint16_t ownPort) { //, std::string dstAddr, uint16_t dstPort) {
  _sockfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  memset(&_locAddr, 0, sizeof(_locAddr));
  _locAddr.sin_family = AF_INET;
  _locAddr.sin_addr.s_addr = INADDR_ANY;
  _locAddr.sin_port = htons(ownPort);
  if (fcntl(_sockfd, F_SETFL, O_ASYNC) < 0) {
    close(_sockfd);
    exit(EXIT_FAILURE);
  }
  /*
  strcpy(_ardupilotIp, dstAddr.c_str());

  memset(&_arupilotAddr, 0, sizeof(_arupilotAddr));
  _arupilotAddr.sin_family = AF_INET;
  _arupilotAddr.sin_addr.s_addr = inet_addr(_ardupilotIp);
  _arupilotAddr.sin_port = htons(dstPort);
  */
}

void GCS::Start() {
  _TxWork();
  _RxWork();
}

void GCS::_TxWork() {}

void GCS::_RxWork() {
  std::thread work([this]() {
    socklen_t fromlen;
    ssize_t recsize;
    while (true) {
      memset(_buf, 0, BUFFER_LENGTH);
      recsize = recvfrom(_sockfd, (void *)_buf, BUFFER_LENGTH, 0,
                         (struct sockaddr *)&_locAddr, &fromlen);

      if (recsize > 0) {
        mavlink_message_t msg;
        mavlink_status_t status;

        for (int i = 0; i < recsize; ++i) {
          if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &status)) {
            // Mavlink Packet Received
            mavlink::MsgMap msgMap(&msg);
            switch (msg.msgid) {
            case msg::HEARTBEAT::MSG_ID: {
              if (false) {
                mavlink::common::msg::HEARTBEAT hb;
                hb.deserialize(msgMap);
                std::string hbStr = hb.to_yaml();
                Info(hbStr);

                switch (hb.type) {
                case (int)MAV_TYPE::SUBMARINE:
                  Info("(Tipo submarino)");
                  break;
                }
                Info("Modos de vuelo permitidos: ");
                if (hb.base_mode & (int)MAV_MODE_FLAG::SAFETY_ARMED)
                  std::cout << "Safety armed" << std::endl;
                if (hb.base_mode & (int)MAV_MODE_FLAG::MANUAL_INPUT_ENABLED)
                  std::cout << "Manual input enabled" << std::endl;
                if (hb.base_mode & (int)MAV_MODE_FLAG::HIL_ENABLED)
                  std::cout << "HIL enabled" << std::endl;
                if (hb.base_mode & (int)MAV_MODE_FLAG::STABILIZE_ENABLED)
                  std::cout << "Stabilize enabled" << std::endl;
                if (hb.base_mode & (int)MAV_MODE_FLAG::GUIDED_ENABLED)
                  std::cout << "Guided enabled" << std::endl;
                if (hb.base_mode & (int)MAV_MODE_FLAG::AUTO_ENABLED)
                  std::cout << "Auto enabled" << std::endl;
                if (hb.base_mode & (int)MAV_MODE_FLAG::TEST_ENABLED)
                  std::cout << "Test enabled" << std::endl;
                if (hb.base_mode & (int)MAV_MODE_FLAG::CUSTOM_MODE_ENABLED)
                  std::cout << "Custom mode enabled" << std::endl;
              }
              break;
            }
            case mavlink::common::msg::MANUAL_CONTROL::MSG_ID: {
              mavlink::common::msg::MANUAL_CONTROL mcm;
              mcm.deserialize(msgMap);
              std::string mcmStr = mcm.to_yaml();
              Debug(mcmStr);
              break;
            }
            case mavlink::common::msg::RAW_IMU::MSG_ID: {
              mavlink::common::msg::RAW_IMU rimsg;
              rimsg.deserialize(msgMap);
              std::string rimsgStr = rimsg.to_yaml();
              Debug(rimsgStr);
              break;
            }
            case msg::LOCAL_POSITION_NED::MSG_ID: {
              msg::LOCAL_POSITION_NED lpn;
              lpn.deserialize(msgMap);
              std::string lpnStr = lpn.to_yaml();
              Debug(lpnStr);
              break;
            }
            case mavlink::common::msg::GLOBAL_POSITION_INT::MSG_ID: {
              msg::GLOBAL_POSITION_INT gpi;
              gpi.deserialize(msgMap);
              std::string gpiStr = gpi.to_yaml();
              Debug(gpiStr);
              break;
            }
            case mavlink::common::msg::RC_CHANNELS_RAW::MSG_ID: {
              msg::RC_CHANNELS_RAW rcov;
              rcov.deserialize(msgMap);
              std::string rcovStr = rcov.to_yaml();
              Debug(rcovStr);
              break;
            }
            case mavlink::common::msg::RC_CHANNELS_OVERRIDE::MSG_ID: {
              msg::RC_CHANNELS_OVERRIDE rcov;
              rcov.deserialize(msgMap);
              std::string rcovStr = rcov.to_yaml();
              Debug(rcovStr);
              break;
            }
            default:
              break;
            }
          }
          std::cout << std::flush;
        }
      }
    }
  });
  work.detach();
}
}
