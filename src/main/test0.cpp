
#include <chrono>
#include <cpplogging/cpplogging.h>
#include <mavlink_cpp/mavlink_cpp.h>
#include <thread>

using namespace mavlink_cpp;
using namespace cpplogging;

int main(void) {
  auto log = CreateLogger("GCS");
  log->SetLogLevel(info);
  log->FlushLogOn(info);

  // std::string ardupilotIp = "192.168.1.101";
  // uint16_t ardupilotPort = 5555;
  uint16_t localPort = 14550;
  Ptr<GCS> control =
      CreateObject<GCS>(localPort); //, ardupilotIp, ardupilotPort);

  control->SetLogName("GCS");
  control->SetLogLevel(info);
  control->FlushLogOn(info);

  control->Start();
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    log->Info("GCS is running");
  }
}
