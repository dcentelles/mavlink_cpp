
#include <chrono>
#include <cpplogging/cpplogging.h>
#include <mavlink_cpp/GCSv1.h>
#include <thread>

using namespace mavlink_cpp;
using namespace cpplogging;
using namespace std::chrono_literals;

static uint16_t localPort = 14550;
static std::shared_ptr<GCSv1> control(new GCSv1(localPort));

int main(void) {
  auto log = CreateLogger("GCS");
  log->SetLogLevel(debug);
  log->FlushLogOn(debug);

  //  uint16_t localPort = 14550;
  //  Ptr<GCS> control = CreateObject<GCS>(localPort);

  control->SetLogName("GCS");
  control->SetLogLevel(debug);
  control->FlushLogOn(debug);

  control->EnableGPSMock(true);
  control->Arm(true);
  control->Start();
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //  while (true) {
  //    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  //    // log->Info("GCS is running");
  //    for (int i = -200; i <= 200; i += 20) {
  //      // control->SetManualControl(i, i, i, i);
  //      control->SetManualControl(0, 0, 506 - i, -23);
  //      std::this_thread::sleep_for(300ms);
  //    }
  //    control->SetManualControl(0, 0, 506, -23);
  //    control->Arm(false);
  //    std::this_thread::sleep_for(3s);
  //    control->Arm(true);

  //    for (int i = -700; i <= 700; i += 20) {
  //      // control->SetManualControl(i, i, i, i);
  //      control->SetManualControl(0, i, 506, -23);
  //      std::this_thread::sleep_for(300ms);
  //    }
  //    control->SetManualControl(0, 0, 506, -23);
  //    std::this_thread::sleep_for(3s);
  //  }
}
