
#include <chrono>
#include <cpplogging/cpplogging.h>
#include <mavlink_cpp/GCS.h>
#include <thread>

using namespace mavlink_cpp;
using namespace cpplogging;
using namespace std::chrono_literals;

static int16_t x, y, z, yaw;

static int16_t x_base = 0, y_base = 0, yaw_base = 0;
static int16_t x_y_yaw_max = 500; // it could be up to 1000 ([-1000, 1000]), but
                                  // we limit to [-500, 500]

static int16_t z_base =
    500; // z range is [0, 1000], but we limit to [z_base - 400, z_base + 400]
static int16_t z_max = 400;

// IMPORTANT NOTE: THE BEHAVIOUR OF THE ROBOT DEPENDS ON THE INPUT GAIN. IT CAN
// BE CHANGED USING THE QGROUNDCONTROL AND THE GAMEPAD. WE RECOMMEND A GAIN OF
// 25%

void StopRobot(const GCSPtr &control) {
  x = x_base;
  y = y_base;
  z = z_base;
  yaw = yaw_base;
  control->SetManualControl(x, y, z, yaw);
}

void ArmRobot(const bool &arm, const GCSPtr &control, const LoggerPtr &log) {
  log->Info("{}", arm ? "Arm" : "Disarm");
  control->Arm(arm);
  while (control->Armed() == arm) {
    std::this_thread::sleep_for(1s);
  }
  log->Info("{}", arm ? "Armed" : "Disarmed");
}

void StopDisarmWaitAndArm(const GCSPtr &control, const LoggerPtr &log) {
  StopRobot(control);
  ArmRobot(false, control, log);
  std::this_thread::sleep_for(5s);
  ArmRobot(true, control, log);
}

int main(void) {
  auto log = CreateLogger("example0");
  log->SetLogLevel(debug);
  log->FlushLogOn(debug);

  uint16_t localPort = 14550;
  GCSPtr control = GCS::Create(localPort);

  control->SetLogName("GCS");
  control->SetLogLevel(info);

  control->Arm(true);
  control->Start();

  control->SetManualMode();
  int it = 0;
  FLY_MODE_R mode;
  std::string mode_str;
  while (true) {
    switch (it % 3) {
    case 0:
      mode = FLY_MODE_R::DEPTH_HOLD;
      mode_str = "DEPTH_HOLD";
      break;
    case 1:
      mode = FLY_MODE_R::STABILIZE;
      mode_str = "STABILIZE";
      break;
    case 2:
      mode = FLY_MODE_R::MANUAL;
      mode_str = "MANUAL";
      break;
    }
    log->Info("Setting nav mode: {}", mode_str);
    control->SetFlyMode(mode);

    while (control->GetCurrentNavMode() != mode)
      std::this_thread::sleep_for(1s);

    log->Info("Setting nav mode {} OK", mode_str);

    StopRobot(control);
    ArmRobot(true, control, log);

    for (int16_t i = -x_y_yaw_max; i <= x_y_yaw_max; i += 20) {
      x = i;
      control->SetManualControl(x, y, z, yaw);
      log->Info("x: {} y: {} z: {} yaw: {}", x, y, z, yaw);
      std::this_thread::sleep_for(300ms);
    }

    StopDisarmWaitAndArm(control, log);

    log->Info("Move y");
    for (int16_t i = -x_y_yaw_max; i <= x_y_yaw_max; i += 20) {
      y = i;
      control->SetManualControl(x, y, z, yaw);
      log->Info("x: {} y: {} z: {} yaw: {}", x, y, z, yaw);
      std::this_thread::sleep_for(300ms);
    }

    StopDisarmWaitAndArm(control, log);

    log->Info("Move z");
    for (int16_t i = -z_max; i <= z_max; i += 20) {
      z = z_base + i;
      control->SetManualControl(x, y, z, yaw);
      log->Info("x: {} y: {} z: {} yaw: {}", x, y, z, yaw);
      std::this_thread::sleep_for(300ms);
    }

    StopDisarmWaitAndArm(control, log);

    log->Info("Move yaw");
    for (int16_t i = -x_y_yaw_max; i <= x_y_yaw_max; i += 20) {
      yaw = i;
      control->SetManualControl(x, y, z, yaw);
      log->Info("x: {} y: {} z: {} yaw: {}", x, y, z, yaw);
      std::this_thread::sleep_for(300ms);
    }

    ArmRobot(false, control, log);

    std::this_thread::sleep_for(3s);
    it++;
  }
}
