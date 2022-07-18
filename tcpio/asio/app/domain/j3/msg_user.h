// Copyright 2020 Horizon Robotics.

#pragma once

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "common/define.h"
#include "image_decoder.h"
#include "hobot-auto/proto_factory.h"

#include <glog/logging.h>

namespace matrix_sample {

using Blocks = std::vector<BlockDese>;
using MsgFrame = std::vector<FrameDese>;
using Handler = std::function<void(std::shared_ptr<google::protobuf::Message>,
                                   const BlockDese &block)>;

class MsgUser {
 public:
  MsgUser() {}

  ~MsgUser() {
    DLOG(INFO) << "stopping J3 communication thread";
    start_thread_ = false;
    wait_cond_.notify_all();
    if (sub_thread_.joinable()) {
      sub_thread_.join();
    }
  }

  void RecvMsg(MsgFrame *frame);
  void HandleMsg();

 private:
  void ProcessMsg();
  void HandleTopic(const Blocks &blocks, const std::string topic,
                   Handler handler);

 private:
  std::queue<MsgFrame> msg_queue_;
  std::condition_variable wait_cond_;
  std::mutex wait_mutex_;
  std::thread sub_thread_;
  std::atomic_bool start_thread_{false};
  hobot_auto::ProtoFactory factory_;
};

}  // namespace matrix_sample
