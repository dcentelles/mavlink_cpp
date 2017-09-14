#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <mavlink_cpp/GCS.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>

namespace mavlink_cpp {

using namespace mavlink;
using namespace mavlink::common;
using namespace std::chrono_literals;

GCS::GCS(uint16_t ownPort) {
  _sockfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  memset(&_locAddr, 0, sizeof(_locAddr));
  _locAddr.sin_family = AF_INET;
  _locAddr.sin_addr.s_addr = INADDR_ANY;
  _locAddr.sin_port = htons(ownPort);
  if (fcntl(_sockfd, F_SETFL, O_NONBLOCK | O_ASYNC) < 0) {
    close(_sockfd);
    exit(EXIT_FAILURE);
  }
  _ardupilotDetected = false;
  if (-1 ==
      bind(_sockfd, (struct sockaddr *)&_locAddr, sizeof(struct sockaddr))) {
    perror("error bind failed");
    close(_sockfd);
    exit(EXIT_FAILURE);
  }
}

void GCS::Start() {
  _RunRxWork();
  std::thread work([this]() {
    std::unique_lock<std::mutex> lock(_ardupilotAddrMutex);
    while (!_ardupilotDetected) {
      _ardupilotAddrCond.wait_for(lock, 2000ms);
      Info("Waiting for the first ardupilot message...");
    }
    lock.unlock();
    Info("Ardupilot detected!");
    _RunHeartBeatWork();
    _RunManualControlWork();
    Info("GCS initialized!");
  });
  work.detach();
}

void GCS::SetManualControl(int16_t x, int16_t y, int16_t z, int16_t r) {
  _manual_control_msg_mutex.lock();
  _manual_control_msg.x = x;
  _manual_control_msg.y = y;
  _manual_control_msg.z = z;
  _manual_control_msg.r = r;
  _manual_control_msg_mutex.unlock();
}

void GCS::_RunManualControlWork() {
  std::thread work([this]() {
    std::string msgstr;
    uint8_t txbuff[BUFFER_LENGTH];
    while (true) {
      int len, bytes_sent;
      memset(txbuff, 0, BUFFER_LENGTH);
      mavlink_message_t auxMsg;
      mavlink::MsgMap msg2send(auxMsg);

      _manual_control_msg_mutex.lock();
      _manual_control_msg.target = 1;
      _manual_control_msg.serialize(msg2send);
      mavlink_finalize_message(&auxMsg, 255, 0, _manual_control_msg.MIN_LENGTH,
                               _manual_control_msg.LENGTH,
                               _manual_control_msg.CRC_EXTRA);
      len = mavlink_msg_to_send_buffer(txbuff, &auxMsg);
      bytes_sent =
          sendto(_sockfd, txbuff, len, 0, (struct sockaddr *)&_ardupilotAddr,
                 sizeof(struct sockaddr_in));
      _manual_control_msg_mutex.unlock();

      msgstr = _manual_control_msg.to_yaml();
      Debug("RunManualControlWork (to: {}): SEND:\n{}", _ardupilotAddr.sin_port,
            msgstr);
      std::this_thread::sleep_for(100ms);
    }
  });
  work.detach();
}

void GCS::SetDepthHoldMode() {
  _manual_control_msg_mutex.lock();
  _manual_control_msg.buttons &= ~MODE_BUTTONS_MASK;
  _manual_control_msg.buttons |= DEPTH_HOLD_BUTTON;
  _manual_control_msg_mutex.unlock();
}

void GCS::SetStabilizeMode() {
  _manual_control_msg_mutex.lock();
  _manual_control_msg.buttons &= ~MODE_BUTTONS_MASK;
  _manual_control_msg.buttons |= STABILIZE_BUTTON;
  _manual_control_msg_mutex.unlock();
}

void GCS::Arm(bool arm) {
  _manual_control_msg_mutex.lock();
  if (arm && !_armed) {
    _manual_control_msg.buttons &= ~DISARM_BUTTON;
    _manual_control_msg.buttons |= ARM_BUTTON;
  } else if (!arm && _armed) {
    _manual_control_msg.buttons &= ~ARM_BUTTON;
    _manual_control_msg.buttons |= DISARM_BUTTON;
  } else {
    _manual_control_msg.buttons &= ~ARM_BUTTON;
    _manual_control_msg.buttons &= ~DISARM_BUTTON;
  }
  _manual_control_msg_mutex.unlock();
}

void GCS::_RunHeartBeatWork() {
  std::thread work([this]() {
    std::string msgstr;
    uint8_t txbuf[BUFFER_LENGTH];
    while (true) {
      mavlink::common::msg::HEARTBEAT hb;
      int len, bytes_sent;
      memset(txbuf, 0, BUFFER_LENGTH);
      hb.type = (int)MAV_TYPE::GCS;
      hb.base_mode = ((int)MAV_MODE_FLAG::MANUAL_INPUT_ENABLED |
                      (int)MAV_MODE_FLAG::SAFETY_ARMED);
      hb.custom_mode = 0;

      mavlink_message_t auxMsg;
      mavlink::MsgMap msg2send(auxMsg);
      hb.serialize(msg2send);

      mavlink::mavlink_finalize_message(&auxMsg, 255, 1, hb.MIN_LENGTH,
                                        hb.LENGTH, hb.CRC_EXTRA);

      len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
      bytes_sent =
          sendto(_sockfd, txbuf, len, 0, (struct sockaddr *)&_ardupilotAddr,
                 sizeof(struct sockaddr_in));
      msgstr = hb.to_yaml();
      Debug("RunHeartBeatWork (to {}):\n{}", _ardupilotAddr.sin_port, msgstr);
      std::this_thread::sleep_for(1s);
    }
  });
  work.detach();
}

void GCS::_RunRxWork() {
  std::thread work([this]() {
    socklen_t fromlen = sizeof(sockaddr);
    ssize_t recsize;
    struct sockaddr_in ardupilotAddr;
    uint8_t rxbuff[BUFFER_LENGTH];
    while (true) {
      memset(rxbuff, 0, BUFFER_LENGTH);
      recsize = recvfrom(_sockfd, (void *)rxbuff, BUFFER_LENGTH, 0,
                         (struct sockaddr *)&ardupilotAddr, &fromlen);

      if (recsize > 0) {
        _ardupilotAddrMutex.lock();
        memcpy(&_ardupilotAddr, &ardupilotAddr, sizeof(_ardupilotAddr));
        _ardupilotDetected = true;
        _ardupilotAddrCond.notify_all();
        _ardupilotAddrMutex.unlock();
        mavlink_message_t msg;
        mavlink_status_t status;
        std::string outputMsg;
        for (int i = 0; i < recsize; ++i) {
          if (mavlink_parse_char(MAVLINK_COMM_0, rxbuff[i], &msg, &status)) {
            // Mavlink Packet Received
            mavlink::MsgMap msgMap(&msg);

            switch (msg.msgid) {
            case msg::HEARTBEAT::MSG_ID: {
              mavlink::common::msg::HEARTBEAT hb;
              hb.deserialize(msgMap);
              std::string hbStr = '\n' + hb.to_yaml();
              outputMsg = hbStr;
              switch (hb.type) {
              case (int)MAV_TYPE::SUBMARINE:
                outputMsg += " (Submarine)\n";
                break;
              }
              outputMsg += "  Fly modes:\n";
              if (hb.base_mode & (int)MAV_MODE_FLAG::SAFETY_ARMED) {
                outputMsg += "   Safety armed.\n";
                _armed = true;
              } else
                _armed = false;

              if (hb.base_mode & (int)MAV_MODE_FLAG::MANUAL_INPUT_ENABLED)
                outputMsg += "   Manual input enabled.\n";
              if (hb.base_mode & (int)MAV_MODE_FLAG::HIL_ENABLED)
                outputMsg += "   HIL enabled.\n";
              if (hb.base_mode & (int)MAV_MODE_FLAG::STABILIZE_ENABLED)
                outputMsg += "   Stabilize enabled.\n";
              if (hb.base_mode & (int)MAV_MODE_FLAG::GUIDED_ENABLED)
                outputMsg += "   Guided enabled.\n";
              if (hb.base_mode & (int)MAV_MODE_FLAG::AUTO_ENABLED)
                outputMsg += "   Auto enabled.\n";
              if (hb.base_mode & (int)MAV_MODE_FLAG::TEST_ENABLED)
                outputMsg += "   Test enabled.\n";
              if (hb.base_mode & (int)MAV_MODE_FLAG::CUSTOM_MODE_ENABLED)
                outputMsg += "   Custom mode enabled.\n";

              Debug(outputMsg);
              break;
            }
            case mavlink::common::msg::MANUAL_CONTROL::MSG_ID: {
              mavlink::common::msg::MANUAL_CONTROL mcm;
              mcm.deserialize(msgMap);
              outputMsg = mcm.to_yaml();
              break;
            }
            case mavlink::common::msg::RAW_IMU::MSG_ID: {
              mavlink::common::msg::RAW_IMU rimsg;
              rimsg.deserialize(msgMap);
              outputMsg = rimsg.to_yaml();
              break;
            }
            case msg::LOCAL_POSITION_NED::MSG_ID: {
              msg::LOCAL_POSITION_NED lpn;
              lpn.deserialize(msgMap);
              outputMsg = lpn.to_yaml();
              break;
            }
            case mavlink::common::msg::GLOBAL_POSITION_INT::MSG_ID: {
              msg::GLOBAL_POSITION_INT gpi;
              gpi.deserialize(msgMap);
              outputMsg = gpi.to_yaml();
              break;
            }
            case mavlink::common::msg::RC_CHANNELS_RAW::MSG_ID: {
              msg::RC_CHANNELS_RAW rcov;
              rcov.deserialize(msgMap);
              outputMsg = rcov.to_yaml();
              break;
            }
            case mavlink::common::msg::RC_CHANNELS_OVERRIDE::MSG_ID: {
              msg::RC_CHANNELS_OVERRIDE rcov;
              rcov.deserialize(msgMap);
              outputMsg = rcov.to_yaml();
              break;
            }
            default:
              break;
            }
          }
        }
      }
    }
  });
  work.detach();
}
}