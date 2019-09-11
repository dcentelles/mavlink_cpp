
#include <chrono>
#include <cpplogging/cpplogging.h>
#include <mavlink_cpp/GCS.h>
#include <thread>

using namespace mavlink_cpp;
using namespace cpplogging;
using namespace std::chrono_literals;

int main(void) {
  auto log = CreateLogger("example0");
  log->SetLogLevel(debug);
  log->FlushLogOn(debug);

  uint16_t localPort = 14550;
  std::shared_ptr<GCS> control(new GCS(localPort));

  control->SetLogName("GCS");
  control->SetLogLevel(info);

  control->Arm(true);
  control->Start();
  int x, y, z, yaw;
  int xyyaw_max = 200;
  int z_base = 500;

  control->SetFlyMode(FLY_MODE_R::STABILIZE);
  control->SetManualMode();
  while (true) {
    x = 0;
    y = 0;
    z = z_base;
    yaw = 0;
    log->Info("Move x");
    for (int i = -xyyaw_max; i <= xyyaw_max; i += 20) {
      x = i;
      control->SetManualControl(x, y, z, yaw);
      log->Info("x: {} y: {} z: {} yaw: {}", x, y, z, yaw);
      std::this_thread::sleep_for(300ms);
    }

    x = 0;
    y = 0;
    z = z_base;
    yaw = 0;
    control->SetManualControl(x, y, z_base, yaw);
    control->Arm(false);
    log->Info("Disarmed");
    std::this_thread::sleep_for(5s);
    log->Info("Armed");
    control->Arm(true);

    log->Info("Move y");
    for (int i = -xyyaw_max; i <= xyyaw_max; i += 20) {
      y = i;
      control->SetManualControl(x, y, z, yaw);
      log->Info("x: {} y: {} z: {} yaw: {}", x, y, z, yaw);
      std::this_thread::sleep_for(300ms);
    }

    std::this_thread::sleep_for(3s);
  }
}
