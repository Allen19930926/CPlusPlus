// Copyright 2020 Horizon Robotics.
#pragma once
#include <functional>
#include <memory>
#include <string>
#include <google/protobuf/message.h>
#include "message/message.h"
#include "hobot-adas-sdk/export.h"
#include "hobot-adas-sdk/msg_types.h"
#ifdef ADAS_JOURNEY
#include "can.h"
#endif
namespace HobotADAS {

#ifdef ADAS_JOURNEY
struct HOBOT_ADAS_SDK_EXPORT CANInput {
  struct CANHeader {
    int64_t time_stamp;
    uint8_t counter;
    uint8_t frame_num;
  };
  CANHeader header;
  std::vector<can_frame> can_frames;
};
#endif

class HOBOT_ADAS_SDK_EXPORT Framework {
 public:
  Framework();
  ~Framework();
  bool Init(std::string config_file_path);
  void Terminate();
  void Start();
  void Stop();
#ifdef ADAS_JOURNEY
  void FeedCAN(CANInput &&can_input);
#endif
  bool RegisterFunction(
    const std::string &topic,
    std::function<void(std::shared_ptr<google::protobuf::Message>)> callback);
  static Framework& Instance();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace HobotADAS
