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
  _lposned_updated = false;
  _gposint_updated = false;
  _gposint_validOrigin = false;
  _scaledImu2_updated = false;
  _enableGPSMock = false;
  _enableManualControl = true;
}

void GCSv1::EnableGPSMock(bool v) {
  std::unique_lock<std::mutex> lock(_enableGPSMock_mutex);
  _enableGPSMock = v;
  _enableGPSMock_cond.notify_all();
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
    if (_enableGPSMock) {
      SendGPSOrigin(0, 0);
      _RunGPSMock();
    }
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
void GCSv1::SendGPSOrigin(uint32_t lat, uint32_t lon) {
  uint8_t txbuff[GCS_BUFFER_LENGTH];
  uint32_t IGNORE_VELOCITIES_AND_ACCURACY =
      (GPS_INPUT_IGNORE_FLAG_ALT | GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
       GPS_INPUT_IGNORE_FLAG_VEL_VERT | GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
       GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
       GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY);

  std::unique_lock<std::mutex> lock(_gposint_mutex);
  _gposint_validOrigin = false;
  memset(txbuff, 0, GCS_BUFFER_LENGTH);
  mavlink_message_t auxMsg;
  _gps_input_msg.gps_id = 0;
  _gps_input_msg.lat = lat;
  _gps_input_msg.lon = lon;
  _gps_input_msg.fix_type = 3;
  _gps_input_msg.hdop = 1;
  _gps_input_msg.vdop = 1;
  _gps_input_msg.satellites_visible = 10;
  _gps_input_msg.ignore_flags = IGNORE_VELOCITIES_AND_ACCURACY;
  mavlink_msg_gps_input_encode(255, 0, &auxMsg, &_gps_input_msg);
  int len = mavlink_msg_to_send_buffer(txbuff, &auxMsg);
  int attempts = 0;
  lock.unlock();

  Info("Reseting GPS Origin...");
  while (attempts < 5) {
    int bytes_sent =
        sendto(_sockfd, txbuff, len, 0, (struct sockaddr *)&_ardupilotAddr,
               sizeof(struct sockaddr_in));
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    attempts += 1;
  }
  lock.lock();
  _gposint_updated = false;
  while (!_gposint_updated) {
    Info("Waiting position estimation from ardupilot...");
    _gposint_cond.wait(lock);
  }
  Info("Position estimation received");
  _gposint_validOrigin = true;
  _gposint_cond.notify_all();
}

void GCSv1::_RunGPSMock() {
  std::thread work([this]() {
    std::string msgstr;
    uint8_t txbuff[GCS_BUFFER_LENGTH];

    uint32_t IGNORE_VELOCITIES_AND_ACCURACY =
        (GPS_INPUT_IGNORE_FLAG_ALT | GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
         GPS_INPUT_IGNORE_FLAG_VEL_VERT | GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
         GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
         GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY);
    while (true) {
      std::unique_lock<std::mutex> lock(_gposint_mutex);
      while (!_gposint_validOrigin) {
        _gposint_cond.wait(lock);
        while (!_gposint_updated)
          _gposint_cond.wait_for(lock, 4000ms);
        if (!_gposint_updated) {
          Warn("GPS position has not been received!");
        }
      }
      memset(txbuff, 0, GCS_BUFFER_LENGTH);
      mavlink_message_t auxMsg;
      _gps_input_msg.gps_id = 0;
      _gps_input_msg.lat = _gposint.lat;
      _gps_input_msg.lon = _gposint.lon;
      _gps_input_msg.fix_type = 3;
      _gps_input_msg.hdop = 0.5;
      _gps_input_msg.vdop = 0.5;
      _gps_input_msg.satellites_visible = 10;
      _gps_input_msg.ignore_flags = IGNORE_VELOCITIES_AND_ACCURACY;
      mavlink_msg_gps_input_encode(255, 0, &auxMsg, &_gps_input_msg);
      int len = mavlink_msg_to_send_buffer(txbuff, &auxMsg);
      // Debug("Sending GPS INPUT");
      int bytes_sent =
          sendto(_sockfd, txbuff, len, 0, (struct sockaddr *)&_ardupilotAddr,
                 sizeof(struct sockaddr_in));
      _gposint_updated = false;
      lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });
  work.detach();
}

void GCSv1::SendGPSInput(mavlink_gps_input_t &msg) {
  uint8_t txbuf[GCS_BUFFER_LENGTH];
  memset(txbuf, 0, GCS_BUFFER_LENGTH);
  mavlink_message_t auxMsg;
  mavlink_msg_gps_input_encode(255, 0, &auxMsg, &msg);
  uint32_t len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
  _socket_mutex.lock();
  uint32_t bytes_sent =
      sendto(_sockfd, txbuf, len, 0, (struct sockaddr *)&_ardupilotAddr,
             sizeof(struct sockaddr_in));
  _socket_mutex.unlock();
}

void GCSv1::SendSetPositionTargetLocalNED(
    mavlink_set_position_target_local_ned_t &msg) {
  uint8_t txbuf[GCS_BUFFER_LENGTH];
  memset(txbuf, 0, GCS_BUFFER_LENGTH);
  mavlink_message_t auxMsg;
  msg.target_system = 1;
  mavlink_msg_set_position_target_local_ned_encode(255, 0, &auxMsg, &msg);
  uint32_t len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
  _socket_mutex.lock();
  uint32_t bytes_sent =
      sendto(_sockfd, txbuf, len, 0, (struct sockaddr *)&_ardupilotAddr,
             sizeof(struct sockaddr_in));
  _socket_mutex.unlock();
}

void GCSv1::EnableManualControl(bool enable) {
  _enableManualControl = enable;
  _enableManualControl_cond.notify_all();
}

void GCSv1::_RunManualControlWork() {
  std::thread work([this]() {
    std::string msgstr;
    uint8_t txbuff[GCS_BUFFER_LENGTH];
    _manual_control_msg.buttons = 0;
    while (true) {
      std::unique_lock<std::mutex> lock(_enableManualControl_mutex);
      while (!_enableManualControl) {
        _enableManualControl_cond.wait(lock);
      }
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
      // Debug("RunManualControlWork (to: {}): SEND:\n{}",
      // _ardupilotAddr.sin_port,
      //      msgstr);
      std::this_thread::sleep_for(100ms);
    }
  });
  work.detach();
}

void GCSv1::SetManualMode() { SetFlyMode(FLY_MODE_R::MANUAL); }

void GCSv1::SetDepthHoldMode() {
  //  _manual_control_msg_mutex.lock();
  //  _manual_control_msg.buttons &= ~MODE_BUTTONS_MASK;
  //  if (_currentMode != FLY_MODE_R::DEPTH_HOLD)
  //    _manual_control_msg.buttons |= DEPTH_HOLD_BUTTON;
  //  _manual_control_msg_mutex.unlock();
  SetFlyMode(FLY_MODE_R::DEPTH_HOLD);
}

void GCSv1::SetStabilizeMode() {
  //  _manual_control_msg_mutex.lock();
  //  _manual_control_msg.buttons &= ~MODE_BUTTONS_MASK;
  //  if (_currentMode != FLY_MODE_R::STABILIZE)
  //    _manual_control_msg.buttons |= STABILIZE_BUTTON;
  //  _manual_control_msg_mutex.unlock();
  SetFlyMode(FLY_MODE_R::STABILIZE);
}

void GCSv1::SetHome(double lat, double lon, double alt) {
  uint8_t txbuf[GCS_BUFFER_LENGTH];
  //    mavlink_command_long_t cmd;
  //    cmd.target_system = 1;
  //    cmd.target_component = 0;
  //    cmd.command = MAV_CMD_DO_SET_HOME;
  //    cmd.param5 = lat * 1e7;
  //    cmd.param6 = lon * 1e7;
  //    cmd.param7 = alt * 1e3;
  //    mavlink_message_t auxMsg;
  //    mavlink_msg_command_long_encode(255, 0, &auxMsg, &cmd);
  //    uint32_t len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
  //    _socket_mutex.lock();
  //    uint32_t bytes_sent =
  //        sendto(_sockfd, txbuf, len, 0, (struct sockaddr *)&_ardupilotAddr,
  //               sizeof(struct sockaddr_in));
  //    _socket_mutex.unlock();

  memset(txbuf, 0, GCS_BUFFER_LENGTH);
  mavlink_set_home_position_t cmd;
  cmd.target_system = 1;
  cmd.latitude = lat * 1e7;
  cmd.longitude = lon * 1e7;
  cmd.altitude = alt * 1e3;
  mavlink_message_t auxMsg;
  mavlink_msg_set_home_position_encode(255, 0, &auxMsg, &cmd);
  uint32_t len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
  _socket_mutex.lock();
  uint32_t bytes_sent =
      sendto(_sockfd, txbuf, len, 0, (struct sockaddr *)&_ardupilotAddr,
             sizeof(struct sockaddr_in));
  _socket_mutex.unlock();
}

void GCSv1::SetFlyMode(FLY_MODE_R flymode) {
  if (_currentMode != flymode) {
    uint8_t txbuf[GCS_BUFFER_LENGTH];
    memset(txbuf, 0, GCS_BUFFER_LENGTH);
    mavlink_set_mode_t cmd;
    cmd.custom_mode = flymode;
    cmd.target_system = 1;
    cmd.base_mode = 209;
    mavlink_message_t auxMsg;
    mavlink_msg_set_mode_encode(255, 0, &auxMsg, &cmd);
    uint32_t len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
    _socket_mutex.lock();
    uint32_t bytes_sent =
        sendto(_sockfd, txbuf, len, 0, (struct sockaddr *)&_ardupilotAddr,
               sizeof(struct sockaddr_in));
    _socket_mutex.unlock();
  }
  //  mavlink_command_long_t cmd;
  //  cmd.command = MAV_CMD_DO_SET_MODE;
  //  //see
  //  https://github.com/mavlink/mavros/blob/master/mavros_msgs/srv/SetMode.srv
  //  //and http://wiki.ros.org/mavros/CustomModes
  //  //and MAV_CMD_DO_SET_MODE in http://mavlink.org/messages/common
  //  cmd.param1 = 209;
  //  cmd.param2 = flymode;
  //  mavlink_message_t auxMsg;
  //  mavlink_msg_command_long_encode(255, 0, &auxMsg, &cmd);
  //  uint32_t len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
  //  _socket_mutex.lock();
  //  uint32_t bytes_sent =
  //      sendto(_sockfd, txbuf, len, 0, (struct sockaddr *)&_ardupilotAddr,
  //             sizeof(struct sockaddr_in));
  //  _socket_mutex.unlock();
}

void GCSv1::Arm(bool arm) {
  if (_enableManualControl) {
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
  } else // SEND MAV_CMD_COMPONENT_ARM_DISARM
  {
    uint8_t txbuf[GCS_BUFFER_LENGTH];
    mavlink_command_long_t cmd;
    cmd.target_system = 1;
    cmd.target_component = 0;
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    if (arm && !_armed) {
      cmd.param1 = 1;
    } else if (!arm && _armed) {
      cmd.param1 = 0;
    }
    mavlink_message_t auxMsg;
    mavlink_msg_command_long_encode(255, 0, &auxMsg, &cmd);
    uint32_t len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
    _socket_mutex.lock();
    uint32_t bytes_sent =
        sendto(_sockfd, txbuf, len, 0, (struct sockaddr *)&_ardupilotAddr,
               sizeof(struct sockaddr_in));
    _socket_mutex.unlock();
  }
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
      // Debug("RunHeartBeatWork (to {}):\n{}", _ardupilotAddr.sin_port,
      // msgstr);
      std::this_thread::sleep_for(1s);
    }
  });
  work.detach();
}

void GCSv1::SendVisionPositionEstimate(
    mavlink_vision_position_estimate_t &msg) {
  uint8_t txbuf[GCS_BUFFER_LENGTH];
  memset(txbuf, 0, GCS_BUFFER_LENGTH);
  mavlink_message_t auxMsg;
  mavlink_msg_vision_position_estimate_encode(255, 0, &auxMsg, &msg);
  uint32_t len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
  _socket_mutex.lock();
  uint32_t bytes_sent =
      sendto(_sockfd, txbuf, len, 0, (struct sockaddr *)&_ardupilotAddr,
             sizeof(struct sockaddr_in));
  _socket_mutex.unlock();
}

void GCSv1::SendGPSFix(mavlink_hil_gps_t &msg) {
  uint8_t txbuf[GCS_BUFFER_LENGTH];
  memset(txbuf, 0, GCS_BUFFER_LENGTH);
  mavlink_message_t auxMsg;
  mavlink_msg_hil_gps_encode(255, 0, &auxMsg, &msg);
  uint32_t len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
  _socket_mutex.lock();
  uint32_t bytes_sent =
      sendto(_sockfd, txbuf, len, 0, (struct sockaddr *)&_ardupilotAddr,
             sizeof(struct sockaddr_in));
  _socket_mutex.unlock();
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
              _hb_mutex.lock();
              mavlink_msg_heartbeat_decode(&msg, &_hb);
              std::string hbStr = "\n"; // + hb.to_yaml();
              outputMsg = hbStr;
              switch (_hb.type) {
              case MAV_TYPE_SUBMARINE:
                outputMsg += " (Submarine)\n";
                break;
              }
              outputMsg += "  Fly modes:\n";
              if (_hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
                outputMsg += "   Safety armed.\n";
                _armed = true;
              } else
                _armed = false;

              if (_hb.base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                outputMsg += "   Manual input enabled.\n";
              if (_hb.base_mode & MAV_MODE_FLAG_HIL_ENABLED)
                outputMsg += "   HIL enabled.\n";
              if (_hb.base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED)
                outputMsg += "   Stabilize enabled.\n";
              if (_hb.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED)
                outputMsg += "   Guided enabled.\n";
              if (_hb.base_mode & MAV_MODE_FLAG_AUTO_ENABLED)
                outputMsg += "   Auto enabled.\n";
              if (_hb.base_mode & MAV_MODE_FLAG_TEST_ENABLED)
                outputMsg += "   Test enabled.\n";
              if (_hb.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
                outputMsg += "   Custom mode enabled.\n";

              _currentMode = static_cast<FLY_MODE_R>(_hb.custom_mode);
              _hb_cb(_hb);
              _hb_updated = true;
              _hb_cond.notify_all();
              _hb_mutex.unlock();

              // Debug(outputMsg);
              break;
            }
            case MAVLINK_MSG_ID_MANUAL_CONTROL: {
              // mavlink::common::msg::MANUAL_CONTROL mcm;
              // cm.deserialize(msgMap);
              // outputMsg = mcm.to_yaml();
              break;
            }
            case MAVLINK_MSG_ID_SCALED_IMU2: {
              _scaledImu2_mutex.lock();
              mavlink_msg_scaled_imu2_decode(&msg, &_scaledImu2);
              // DebugScaledIMU2(_scaledImu2);
              _scaledImu2_cb(_scaledImu2);
              _scaledImu2_updated = true;
              _scaledImu2_cond.notify_all();
              _scaledImu2_mutex.unlock();
              break;
            }
            case MAVLINK_MSG_ID_RAW_IMU: {
              _rawImu_mutex.lock();
              mavlink_msg_raw_imu_decode(&msg, &_rawImu);
              // DebugRawIMU(_rawImu);
              _rawImu_updated = true;
              _rawImu_cond.notify_all();
              _rawImu_mutex.unlock();
              break;
            }
            case MAVLINK_MSG_ID_VFR_HUD: {
              _vfr_hud_mutex.lock();
              mavlink_msg_vfr_hud_decode(&msg, &_vfr_hud);
              // DebugRawIMU(_rawImu);
              _vfr_hud_cb(_vfr_hud);
              _vfr_hud_updated = true;
              _vfr_hud_cond.notify_all();
              _vfr_hud_mutex.unlock();
              break;
            }
            case MAVLINK_MSG_ID_ATTITUDE: {
              _attitude_mutex.lock();
              mavlink_msg_attitude_decode(&msg, &_attitude);
              // DebugAttitude(_attitude);
              _attitude_cb(_attitude);
              _attitude_updated = true;
              _attitude_cond.notify_all();
              _attitude_mutex.unlock();
              break;
            }
            case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
              _attitudeQuaternion_mutex.lock();
              mavlink_msg_attitude_quaternion_decode(&msg,
                                                     &_attitudeQuaternion);
              // DebugAttitudeQuaternion(_attitudeQuaternion);
              _attitudeQuaternion_updated = true;
              _attitudeQuaternion_cond.notify_all();
              _attitudeQuaternion_mutex.unlock();
              break;
            }
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
              _gposint_mutex.lock();
              mavlink_msg_global_position_int_decode(&msg, &_gposint);
              DebugGlobalPositionInt(_gposint);
              _gposint_cb(_gposint);
              _gposint_updated = true;
              _gposint_cond.notify_all();
              _gposint_mutex.unlock();
              break;
            }
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
              _lposned_mutex.lock();
              mavlink_msg_local_position_ned_decode(&msg, &_lposned);
              DebugLocalPositonNED(_lposned);
              _lposned_cb(_lposned);
              _lposned_updated = true;
              _lposned_cond.notify_all();
              _lposned_mutex.unlock();
              break;
            }
            case MAVLINK_MSG_ID_HOME_POSITION: {
              _home_mutex.lock();
              mavlink_msg_home_position_decode(&msg, &_home);
              _home_cb(_home);
              _home_updated = true;
              _home_position_set = true;
              _home_cond.notify_all();
              _home_mutex.unlock();
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
void GCSv1::DebugGlobalPositionInt(mavlink_global_position_int_t &msg) {
  Debug("GLOBAL_POSITION_INT:"
        "\ttime_boot_ms: {}\n"
        "\tlat: {}\n"
        "\tlon: {}\n"
        "\talt: {}\n"
        "\trelative_alt {}\n"
        "\tvx: {}\n"
        "\tvy: {}\n"
        "\tvz: {}\n"
        "\thdg: {}\n",
        msg.time_boot_ms, msg.lat, msg.lon, msg.alt, msg.relative_alt, msg.vx,
        msg.vy, msg.vz, msg.hdg);
}
void GCSv1::DebugLocalPositonNED(mavlink_local_position_ned_t &msg) {
  Debug("LOCAL_POSITION_NED:"
        "\ttime_boot_ms: {}\n"
        "\tx: {}\n"
        "\ty: {}\n"
        "\tz: {}\n"
        "\tvx: {}\n"
        "\tvy: {}\n"
        "\tvz: {}\n",
        msg.time_boot_ms, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz);
}
void GCSv1::WaitForNEDUpdate() {
  std::unique_lock<std::mutex> lock(_lposned_mutex);
  while (!_lposned_updated && !_gposint_validOrigin) {
    _lposned_cond.wait(lock);
  }
}
mavlink_global_position_int_t GCSv1::GetGPS() { return _gposint; }
mavlink_local_position_ned_t GCSv1::GetNED() { return _lposned; }
mavlink_scaled_imu2_t GCSv1::GetScaledIMU2() { return _scaledImu2; }

mavlink_attitude_quaternion_t GCSv1::GetAttitudeQuaternion() {
  return _attitudeQuaternion;
}
mavlink_attitude_t GCSv1::GetAttitude() { return _attitude; }
mavlink_vfr_hud_t GCSv1::GetVfrHud() {
  _vfr_hud_mutex.lock();
  mavlink_vfr_hud_t res = _vfr_hud;
  _vfr_hud_mutex.unlock();
  return res;
}

void GCSv1::DebugScaledIMU2(mavlink_scaled_imu2_t &msg) {
  Debug("SCALED_IMU:"
        "\ttime_boot_ms: {}\n"
        "\txacc: {}\n"
        "\tyacc: {}\n"
        "\tzacc: {}\n"
        "\txgyro: {}\n"
        "\tygyro: {}\n"
        "\tzgyro: {}\n"
        "\txmag: {}\n"
        "\tymag: {}\n"
        "\tzmag: {}\n",
        msg.time_boot_ms, msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro,
        msg.zgyro, msg.xmag, msg.ymag, msg.zmag);
}
void GCSv1::DebugRawIMU(mavlink_raw_imu_t &msg) {
  Debug("RAW_IMU:"
        "\ttime_usec: {}\n"
        "\txacc: {}\n"
        "\tyacc: {}\n"
        "\tzacc: {}\n"
        "\txgyro: {}\n"
        "\tygyro: {}\n"
        "\tzgyro: {}\n"
        "\txmag: {}\n"
        "\tymag: {}\n"
        "\tzmag: {}\n",
        msg.time_usec, msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro,
        msg.zgyro, msg.xmag, msg.ymag, msg.zmag);
}

void GCSv1::DebugAttitudeQuaternion(mavlink_attitude_quaternion_t &msg) {
  Debug("ATTITUDE_QUATERNION:"
        "\ttime_boot_ms: {}\n"
        "\tq1: {}\n"
        "\tq2: {}\n"
        "\tq3: {}\n"
        "\tq4: {}\n"
        "\trollspeed: {}\n"
        "\tpitchspeed: {}\n"
        "\tyawspeed: {}\n",
        msg.time_boot_ms, msg.q1, msg.q2, msg.q3, msg.q4, msg.rollspeed,
        msg.pitchspeed, msg.yawspeed);
}

void GCSv1::DebugAttitude(mavlink_attitude_t &msg) {
  Debug("ATTITUDE:"
        "\ttime_boot_ms: {}\n"
        "\troll: {}\n"
        "\tpitch: {}\n"
        "\tyaw: {}\n"
        "\trollspeed: {}\n"
        "\tpitchspeed: {}\n"
        "\tyawspeed: {}\n",
        msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw, msg.rollspeed,
        msg.pitchspeed, msg.yawspeed);
}

void GCSv1::SetHeartbeatCb(
    std::function<void(const mavlink_heartbeat_t &)> handler) {
  _hb_cb = handler;
}
void GCSv1::SetAttitudeCb(
    std::function<void(const mavlink_attitude_t &)> handler) {
  _attitude_cb = handler;
}
void GCSv1::SetVfrHudCb(
    std::function<void(const mavlink_vfr_hud_t &)> handler) {
  _vfr_hud_cb = handler;
}
void GCSv1::SetLocalPositionNEDCb(
    std::function<void(const mavlink_local_position_ned_t &)> handler) {
  _lposned_cb = handler;
}
void GCSv1::SetGlobalPositionInt(
    std::function<void(const mavlink_global_position_int_t &)> handler) {
  _gposint_cb = handler;
}
void GCSv1::SetSclaedIMU2(
    std::function<void(const mavlink_scaled_imu2_t &)> handler) {
  _scaledImu2_cb = handler;
}
void GCSv1::SetHomeUpdatedCb(
    std::function<void(const mavlink_home_position_t &)> handler) {
  _home_cb = handler;
}
} // namespace mavlink_cpp
