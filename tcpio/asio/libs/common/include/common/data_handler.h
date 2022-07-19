// Copyright 2020 Horizon Robotics.
#ifndef SRC_COMMUNICATION_USER_DATA_HANDLER_H_
#define SRC_COMMUNICATION_USER_DATA_HANDLER_H_

#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "common/define.h"
#include "message/proto/proto_msg.hpp"
#include "pack-sdk/pack_sdk.h"
#include "protocol/frame.v2.pb.h"

namespace matrix_sample {

class DataHandler {
 public:
  static DataHandler &Instance() {
    static DataHandler instance;
    return instance;
  }

 public:
  void CreateBlockData(
      const std::shared_ptr<hobot::message::ProtoMsg<FrameV2Proto::Frame> >
          &frame_msg,
      std::vector<uint8_t> *meta, std::vector<uint8_t *> *data,
      std::vector<int> *data_size);

  void HandleFrame(const PackFrame &frame, std::vector<FrameDese> *dese);

 private:
  void Deduplication(char **list, int size, std::set<std::string> *set);

 private:
  DataHandler() = default;
};

#endif  // SRC_COMMUNICATION_USER_DATA_HANDLER_H_
}  // namespace matrix_sample
