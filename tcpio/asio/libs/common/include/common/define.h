// Copyright 2020 Horizon Robotics.

#ifndef SRC_COMMUNICATION_USER_DEFINE_H_
#define SRC_COMMUNICATION_USER_DEFINE_H_

#include <string.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace matrix_sample {

const char conn_content[] =
    "{\"discovery_pattern\":1,\"participants\":[{\"id\":1,\"protocol\""
    ":\"zmq_tcp\",\"link_info\":\"127.0.0.1:8890\",\"link_type\":\"\"}]}";

struct BlockDese {
  BlockDese()
      : gen_ts_(0),
        done_ts_(0),
        meta_(std::make_shared<std::vector<uint8_t>>()),
        meta_size_(0),
        data_(std::make_shared<std::vector<uint8_t>>()),
        data_size_(0) {}

  BlockDese(int64_t gts, int64_t dts, const uint8_t *meta, uint64_t mSize,
            const uint8_t *data, uint64_t dSize)
      : BlockDese() {
    gen_ts_ = gts;
    done_ts_ = dts;

    if (meta != nullptr && mSize > 0) {
      meta_size_ = mSize;
      meta_->resize(mSize);
      memcpy(meta_->data(), meta, mSize);
    }

    if (data != nullptr && dSize > 0) {
      data_size_ = dSize;
      data_->resize(dSize);
      memcpy(data_->data(), data, dSize);
    }
  }
  ~BlockDese() {}

  BlockDese(const BlockDese &b) = default;
  BlockDese &operator=(const BlockDese &b) = default;
  BlockDese(BlockDese &&b) = default;
  BlockDese &operator=(BlockDese &&b) = default;

  int64_t gen_ts_;
  int64_t done_ts_;
  std::shared_ptr<std::vector<uint8_t>> meta_;
  uint64_t meta_size_;
  std::shared_ptr<std::vector<uint8_t>> data_;
  uint64_t data_size_;
};

typedef struct FrameDese {
  std::string topic;
  std::vector<BlockDese> block;
} FrameDese;

}  // namespace matrix_sample
#endif  // SRC_COMMUNICATION_USER_DEFINE_H_
