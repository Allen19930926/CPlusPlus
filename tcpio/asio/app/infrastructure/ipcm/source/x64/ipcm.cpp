
#include "ipcm.h"


void IpcM::Run() {
  if (start_thread_) {
    return;
  }
  
  start_thread_ = true;
  // ipcm_thread_ = std::thread([this]() {
  //   while (start_thread_) {
  //     std::chrono::milliseconds timespan(50);
  //     std::this_thread::sleep_for(timespan);
  //   }
  // });
}

// static void IpcM_Appl_rxCallback(IpcM_FrameTypes_T frameType, const uint8 * data, const uint32 len)
// {
// }

int IpcM::Write(const IpcM_FrameTypes_T frameType, const void* srcAddress, const uint16 msgLen) {
    return 0;
}
