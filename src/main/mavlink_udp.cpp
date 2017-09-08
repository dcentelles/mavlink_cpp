/*******************************************************************************
 Copyright (C) 2010  Bryan Godbolt godbolt ( a t ) ualberta.ca

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ****************************************************************************/
/*
 This program sends some data to qgroundcontrol using the mavlink protocol.  The
 sent packets
 cause qgroundcontrol to respond with heartbeats.  Any settings or custom
 commands sent from
 qgroundcontrol are printed by this program along with the heartbeats.


 I compiled this program sucessfully on Ubuntu 10.04 with the following command

 gcc -I ../../pixhawk/mavlink/include -o udp-server udp-server-test.c

 the rt library is needed for the clock_gettime on linux
 */
/* These headers are for QNX, but should all be standard on unix/linux */
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <arpa/inet.h>
#include <stdbool.h> /* required for the definition of bool in C99 */
#include <sys/time.h>
#include <time.h>
#endif

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */

#include <mavlink_cpp/mavlink/common/common.hpp>
#include <mavlink_cpp/utils.h>
#include <thread>
using namespace mavlink;
using namespace mavlink::common;

#define BUFFER_LENGTH                                                          \
  2041 // minimum buffer size that can be used with qnx (I don't know why)

uint64_t microsSinceEpoch();
#include <iostream>

int main(int argc, char *argv[]) {
  char help[] = "--help";
  printf("Hello World");

  // std::cout << "Hello World" << std::endl;

  char target_ip[100];

  float position[6] = {};
  int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  struct sockaddr_in gcAddr;
  struct sockaddr_in locAddr;
  // struct sockaddr_in fromAddr;
  uint8_t buf[BUFFER_LENGTH];
  ssize_t recsize;
  socklen_t fromlen;
  int bytes_sent;
  mavlink_message_t msg;
  uint16_t len;
  int i = 0;
  // int success = 0;
  unsigned int temp = 0;

  // Check if --help flag was used
  if ((argc == 2) && (strcmp(argv[1], help) == 0)) {
    printf("\n");
    printf("\tUsage:\n\n");
    printf("\t");
    printf("%s", argv[0]);
    printf(" <ip address of QGroundControl>\n");
    printf("\tDefault for localhost: udp-server 127.0.0.1\n\n");
    exit(EXIT_FAILURE);
  }

  // Change the target ip if parameter was given
  strcpy(target_ip, "127.0.0.1");
  if (argc == 2) {
    strcpy(target_ip, argv[1]);
  }

  memset(&locAddr, 0, sizeof(locAddr));
  locAddr.sin_family = AF_INET;
  locAddr.sin_addr.s_addr = INADDR_ANY;
  locAddr.sin_port = htons(14557);

  if (-1 == bind(sock, (struct sockaddr *)&locAddr, sizeof(struct sockaddr))) {
    perror("error bind failed");
    close(sock);
    exit(EXIT_FAILURE);
  }

/* Attempt to make it non blocking */
#if (defined __QNX__) | (defined __QNXNTO__)
  if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
#else
  if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
#endif
  {
    fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
    close(sock);
    exit(EXIT_FAILURE);
  }

  memset(&gcAddr, 0, sizeof(gcAddr));
  gcAddr.sin_family = AF_INET;
  gcAddr.sin_addr.s_addr = inet_addr(target_ip);
  // gcAddr.sin_port = htons(14555);
  gcAddr.sin_port = htons(14550);

  std::thread work([sock, gcAddr]() {

    while (false) {
      uint8_t txbuf[BUFFER_LENGTH];
      mavlink::common::msg::RC_CHANNELS_OVERRIDE rcov;
      int len, bytes_sent;
      memset(txbuf, 0, BUFFER_LENGTH);
      rcov.chan1_raw = 1400;
      rcov.chan2_raw = 1500;
      rcov.chan3_raw = 1500;
      rcov.chan4_raw = 1500;
      rcov.chan5_raw = 1500;
      rcov.chan6_raw = 1500;
      rcov.chan7_raw = 1500;

      mavlink_message_t auxMsg;
      mavlink::MsgMap msg2send(auxMsg);
      rcov.serialize(msg2send);

      mavlink::mavlink_finalize_message(&auxMsg, 255, 0, rcov.MIN_LENGTH,
                                        rcov.LENGTH, rcov.CRC_EXTRA);
      len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
      bytes_sent = sendto(sock, txbuf, len, 0, (struct sockaddr *)&gcAddr,
                          sizeof(struct sockaddr_in));
      fprintf(stdout, "\n(%ld): enviado heartbeat\n",
              (uint64_t)(microsSinceEpoch() / 1000));
      std::cout << std::flush;
      sleep(1); // Sleep one second
    }

    while (true) {

      uint8_t txbuf[BUFFER_LENGTH];
      mavlink::common::msg::HEARTBEAT hb;
      int len, bytes_sent;
      memset(txbuf, 0, BUFFER_LENGTH);
      hb.type = (int)MAV_TYPE::SUBMARINE;
      hb.autopilot = (int)MAV_AUTOPILOT::GENERIC;
      hb.base_mode = (int)MAV_MODE::MANUAL_ARMED;
      // hb.base_mode = (int)MAV_MODE::GUIDED_ARMED;
      hb.custom_mode = 0;
      hb.system_status = (int)MAV_STATE::ACTIVE;
      // hb.mavlink_version = (uint8_t)MAV_PROTOCOL_CAPABILITY::MAVLINK2;

      mavlink_message_t auxMsg;
      mavlink::MsgMap msg2send(auxMsg);
      hb.serialize(msg2send);

      mavlink::mavlink_finalize_message(&auxMsg, 1, 1, hb.MIN_LENGTH, hb.LENGTH,
                                        hb.CRC_EXTRA);

      len = mavlink_msg_to_send_buffer(txbuf, &auxMsg);
      bytes_sent = sendto(sock, txbuf, len, 0, (struct sockaddr *)&gcAddr,
                          sizeof(struct sockaddr_in));
      fprintf(stdout, "\n(%ld): enviado heartbeat: %s\n",
              (uint64_t)(microsSinceEpoch() / 1000), hb.to_yaml().c_str());
      std::cout << std::flush;
      sleep(1); // Sleep one second
    }
  });

  for (;;) {
    memset(buf, 0, BUFFER_LENGTH);
    recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0,
                       (struct sockaddr *)&gcAddr, &fromlen);

    if (recsize > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;

      for (i = 0; i < recsize; ++i) {
        temp = buf[i];
        std::cout << std::flush;
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
          // Mavlink Packet Received
          mavlink::MsgMap msgMap(&msg);
          bool print = true;
          // std::cout << "ID: " << msg.msgid << std::endl;
          switch (msg.msgid) {
          case msg::HEARTBEAT::MSG_ID: {
            if (true) {
              //              std::cout << "MANUAL CONTROL ID: "
              //                        <<
              //                        mavlink::common::msg::MANUAL_CONTROL::MSG_ID
              //                        << std::endl;
              mavlink::common::msg::HEARTBEAT hb;
              hb.deserialize(msgMap);
              std::string hbStr = hb.to_yaml();
              fprintf(stdout, "\n%s\n", hbStr.c_str());

              switch (hb.type) {
              case (int)MAV_TYPE::SUBMARINE:
                std::cout << "(Tipo submarino)" << std::endl;
                break;
              }
              std::cout << std::endl
                        << "Modos de vuelo permitidos: " << std::endl;
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
            // fprintf(stdout, "\n%s\n", mcmStr.c_str());
            break;
          }
          case mavlink::common::msg::RAW_IMU::MSG_ID: {
            mavlink::common::msg::RAW_IMU rimsg;
            rimsg.deserialize(msgMap);
            std::string rimsgStr = rimsg.to_yaml();
            // fprintf(stdout, "\n%s\n", rimsgStr.c_str ());
            break;
          }
          case msg::LOCAL_POSITION_NED::MSG_ID: {
            msg::LOCAL_POSITION_NED lpn;
            lpn.deserialize(msgMap);
            std::string lpnStr = lpn.to_yaml();
            // mav
            // fprintf(stdout, "\n%s\n", lpnStr.c_str ());
            break;
          }
          case mavlink::common::msg::GLOBAL_POSITION_INT::MSG_ID: {
            msg::GLOBAL_POSITION_INT gpi;
            gpi.deserialize(msgMap);
            std::string gpiStr = gpi.to_yaml();
            // mav
            // fprintf(stdout, "\n%s\n", gpiStr.c_str ());
            break;
          }
          case mavlink::common::msg::RC_CHANNELS_RAW::MSG_ID: {
            msg::RC_CHANNELS_RAW rcov;
            rcov.deserialize(msgMap);
            std::string rcovStr = rcov.to_yaml();
            // mav
            fprintf(stdout, "\n%s\n", rcovStr.c_str());
            break;
          }
          case mavlink::common::msg::RC_CHANNELS_OVERRIDE::MSG_ID: {
            msg::RC_CHANNELS_OVERRIDE rcov;
            rcov.deserialize(msgMap);
            std::string rcovStr = rcov.to_yaml();
            // mav
            fprintf(stdout, "\n%s\n", rcovStr.c_str());
            break;
          }
          default:
            break;
          }

          //          if (true)
          //            fprintf(stdout, "\n(%ld): received packet: SYS: %d,
          //            COMP: %d, LEN: "
          //                            "%d, MSG ID: %d\n",
          //                    (uint64_t)(microsSinceEpoch() / 1000),
          //                    msg.sysid,
          //                    msg.compid, msg.len, msg.msgid);
          //          std::cout << std::flush;
        }
      }
      // printf("\n");
      std::cout << std::flush;
    }
    memset(buf, 0, BUFFER_LENGTH);
  }
}

/* QNX timer version */
#if (defined __QNX__) | (defined __QNXNTO__)
uint64_t microsSinceEpoch() {

  struct timespec time;

  uint64_t micros = 0;

  clock_gettime(CLOCK_REALTIME, &time);
  micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec / 1000;

  return micros;
}
#else
uint64_t microsSinceEpoch() {

  struct timeval tv;

  uint64_t micros = 0;

  gettimeofday(&tv, NULL);
  micros = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

  return micros;
}
#endif

/*
//Send Heartbeat
mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER,
MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
len = mavlink_msg_to_send_buffer(buf, &msg);
bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct
sockaddr_in));

// Send Status
mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0,
0, 0, 0);
len = mavlink_msg_to_send_buffer(buf, &msg);
bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct
sockaddr_in));

// Send Local Position
mavlink_msg_local_position_ned_pack(1, 200, &msg, microsSinceEpoch(),
                                                                position[0],
position[1], position[2],
                                                                position[3],
position[4], position[5]);
len = mavlink_msg_to_send_buffer(buf, &msg);
bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct
sockaddr_in));

// Send attitude
mavlink_msg_attitude_pack(1, 200, &msg, microsSinceEpoch(), 1.2, 1.7, 3.14,
0.01, 0.02, 0.03);
len = mavlink_msg_to_send_buffer(buf, &msg);
bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct
sockaddr_in));

*/
