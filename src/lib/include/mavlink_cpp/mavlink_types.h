#ifndef MAVLINK_CPP_TYPES_H
#define MAVLINK_CPP_TYPES_H

namespace mavlink_cpp {
enum FLY_MODE_R { // Fly mode in the 'custom_mode' field of the autopilot
                  // heartbeat
  STABILIZE = 0,
  DEPTH_HOLD = 2,
  MANUAL = 19,
  GUIDED = 4,
  POS_HOLD = 16
};
}
#endif // MAVLINK_CPP_TYPES_H
