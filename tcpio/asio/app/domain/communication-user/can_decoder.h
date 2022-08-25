// Copyright 2020 Horizon Robotics.

#pragma once

#include <iostream>
#include <string>

#include "common/can.h"

namespace matrix_sample {

class CanDecoder : public HobotADAS::CAN::CANParser {
 public:
  static CanDecoder &Instance() {
    static CanDecoder instance;
    return instance;
  }

 public:
  bool InitDbc(const std::string json_file);
  void FeedInfo(int idx, int dlc, uint8_t data[8], int64_t ts);
  void SetCanDbcStatus(bool status);
  bool GetCanDbcStatus();

 private:
  CanDecoder() = default;

 private:
  bool init_can_dbc_status = false;
};

}  // namespace matrix_sample
