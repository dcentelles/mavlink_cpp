#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <fcntl.h>

#include <mavlink_cpp/GCSv1.h>
//#include <mavlink_cpp/mavlink/v2_cpp/common/common.hpp>

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

using namespace std::chrono_literals;

GCSv1::GCSv1(uint16_t ownPort) {
  _sockfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  memset(&_locAddr, 0, sizeof(_locAddr));
  _locAddr.sin_family = AF_INET;
  _locAddr.sin_addr.s_addr = INADDR_ANY;
  _locAddr.sin_port = htons(ownPort);
  if (fcntl(_sockfd, F_SETFL, O_ASYNC) < 0) {
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

void GCSv1::Start() {
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

void GCSv1::SetManualControl(int16_t x, int16_t y, int16_t z, int16_t r) {
  _manual_control_msg_mutex.lock();
  _manual_control_msg.x = x;
  _manual_control_msg.y = y;
  _manual_control_msg.z = z;
  _manual_control_msg.r = r;
  _manual_control_msg_mutex.unlock();
}

void GCSv1::_RunManualControlWork() {
  std::thread work([this]() {
    std::string msgstr;
    uint8_t txbuff[GCS_BUFFER_LENGTH];
    while (true) {
      int len, bytes_sent;
      memset(txbuff, 0, GCS_BUFFER_LENGTH);
      mavlink_message_t auxMsg;

      _manual_control_msg_mutex.lock();
      _manual_control_msg.target = 1;

      mavlink_msg_manual_control_encode(255, 0, &auxMsg, &_manual_control_msg);
      len = mavlink_msg_to_send_buffer(txbuff, &auxMsg);
      _socket_mutex.lock();
      bytes_sent =
          sendto(_sockfd, txbuff, len, 0, (struct sockaddr *)&_ardupilotAddr,
                 sizeof(struct sockaddr_in));
      _socket_mutex.unlock();
      _manual_control_msg_mutex.unlock();

      msgstr = "";
      Debug("RunManualControlWork (to: {}): SEND:\n{}", _ardupilotAddr.sin_port,
            msgstr);
      std::this_thread::sleep_for(100ms);
    }
  });
  work.detach();
}

void GCSv1::SetManualMode() {
  if (_currentMode != FLY_MODE_R::MANUAL)
    _OrderManualMode();
}

void GCSv1::_OrderManualMode() {
  std::string msgstr;
  uint8_t txbuf[GCS_BUFFER_LENGTH];
  int len, bytes_sent;
  memset(txbuf, 0, GCS_BUFFER_LENGTH);
  mavlink_set_mode_t cmd;
  cmd.custom_mode = 19;
  cmd.target_system = 1;
  cmd.base_mode = 209;
  mavlink_message_t auxMsg;
  mavlink_msg_set_mode_encode(255, 0, &auxMsg, &cmd);
  len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
  _socket_mutex.lock();
  bytes_sent =
      sendto(_sockfd, txbuf, len, 0, (struct sockaddr *)&_ardupilotAddr,
             sizeof(struct sockaddr_in));
  _socket_mutex.unlock();
  msgstr = "";
  Debug("OrderManualMode: (to {}):\n{}", _ardupilotAddr.sin_port, msgstr);
}

void GCSv1::SetDepthHoldMode() {
  _manual_control_msg_mutex.lock();
  _manual_control_msg.buttons &= ~MODE_BUTTONS_MASK;
  if (_currentMode != FLY_MODE_R::DEPTH_HOLD)
    _manual_control_msg.buttons |= DEPTH_HOLD_BUTTON;
  _manual_control_msg_mutex.unlock();
}

void GCSv1::SetStabilizeMode() {
  _manual_control_msg_mutex.lock();
  _manual_control_msg.buttons &= ~MODE_BUTTONS_MASK;
  if (_currentMode != FLY_MODE_R::STABILIZE)
    _manual_control_msg.buttons |= STABILIZE_BUTTON;
  _manual_control_msg_mutex.unlock();
}

void GCSv1::Arm(bool arm) {
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

void GCSv1::_RunHeartBeatWork() {
  std::thread work([this]() {
    std::string msgstr;
    uint8_t txbuf[GCS_BUFFER_LENGTH];
    while (true) {
      mavlink_heartbeat_t hb;
      int len, bytes_sent;
      memset(txbuf, 0, GCS_BUFFER_LENGTH);
      hb.custom_mode = 0;
      hb.type = MAV_TYPE_GCS;
      hb.autopilot = 8;
      hb.base_mode = ((int)MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                      (int)MAV_MODE_FLAG_SAFETY_ARMED);

      hb.system_status = 4;
      hb.mavlink_version = 3;

      mavlink_message_t auxMsg;
      mavlink_msg_heartbeat_encode(255, 0, &auxMsg, &hb);
      len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
      _socket_mutex.lock();
      bytes_sent =
          sendto(_sockfd, txbuf, len, 0, (struct sockaddr *)&_ardupilotAddr,
                 sizeof(struct sockaddr_in));
      _socket_mutex.unlock();
      msgstr = "";
      Debug("RunHeartBeatWork (to {}):\n{}", _ardupilotAddr.sin_port, msgstr);
      std::this_thread::sleep_for(1s);
    }
  });
  work.detach();
}

void GCSv1::_RunRxWork() {
  std::thread work([this]() {
    socklen_t fromlen = sizeof(sockaddr);
    ssize_t recsize;
    struct sockaddr_in ardupilotAddr;
    uint8_t rxbuff[GCS_BUFFER_LENGTH];
    while (true) {
      memset(rxbuff, 0, GCS_BUFFER_LENGTH);
      recsize = recvfrom(_sockfd, (void *)rxbuff, GCS_BUFFER_LENGTH, 0,
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

            switch (msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT: {
              mavlink_heartbeat_t hb;
              mavlink_msg_heartbeat_decode(&msg, &hb);
              std::string hbStr = "\n"; // + hb.to_yaml();
              outputMsg = hbStr;
              switch (hb.type) {
              case MAV_TYPE_SUBMARINE:
                outputMsg += " (Submarine)\n";
                break;
              }
              outputMsg += "  Fly modes:\n";
              if (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
                outputMsg += "   Safety armed.\n";
                _armed = true;
              } else
                _armed = false;

              if (hb.base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                outputMsg += "   Manual input enabled.\n";
              if (hb.base_mode & MAV_MODE_FLAG_HIL_ENABLED)
                outputMsg += "   HIL enabled.\n";
              if (hb.base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED)
                outputMsg += "   Stabilize enabled.\n";
              if (hb.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED)
                outputMsg += "   Guided enabled.\n";
              if (hb.base_mode & MAV_MODE_FLAG_AUTO_ENABLED)
                outputMsg += "   Auto enabled.\n";
              if (hb.base_mode & MAV_MODE_FLAG_TEST_ENABLED)
                outputMsg += "   Test enabled.\n";
              if (hb.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
                outputMsg += "   Custom mode enabled.\n";

              switch (hb.custom_mode) {
              case FLY_MODE_R::MANUAL:
                _currentMode = FLY_MODE_R::MANUAL;
                break;
              case FLY_MODE_R::STABILIZE:
                _currentMode = FLY_MODE_R::STABILIZE;
                break;
              case FLY_MODE_R::DEPTH_HOLD:
                _currentMode = FLY_MODE_R::DEPTH_HOLD;
                break;
              default:
                break;
              }

              Debug(outputMsg);
              break;
            }
            case MAVLINK_MSG_ID_MANUAL_CONTROL: {
              // mavlink::common::msg::MANUAL_CONTROL mcm;
              // cm.deserialize(msgMap);
              // outputMsg = mcm.to_yaml();
              break;
            }
            case MAVLINK_MSG_ID_RAW_IMU: {
              // mavlink::common::msg::RAW_IMU rimsg;
              // rimsg.deserialize(msgMap);
              // outputMsg = rimsg.to_yaml();
              break;
            }
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
              // msg::LOCAL_POSITION_NED lpn;
              // lpn.deserialize(msgMap);
              // outputMsg = lpn.to_yaml();
              break;
            }
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
              // msg::GLOBAL_POSITION_INT gpi;
              // gpi.deserialize(msgMap);
              // outputMsg = gpi.to_yaml();
              break;
            }
            case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
              // msg::RC_CHANNELS_RAW rcov;
              // rcov.deserialize(msgMap);
              // outputMsg = rcov.to_yaml();
              break;
            }
            case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
              // msg::RC_CHANNELS_OVERRIDE rcov;
              // rcov.deserialize(msgMap);
              // outputMsg = rcov.to_yaml();
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
