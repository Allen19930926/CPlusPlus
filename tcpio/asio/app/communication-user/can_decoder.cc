// Copyright 2021 Horizon Robotics.

#include "can_decoder.h"

bool matrix_sample::CanDecoder::InitDbc(const std::string json_file) {
  return HobotADAS::CAN::CANParser::Init(json_file);
}
void matrix_sample::CanDecoder::SetCanDbcStatus(bool status) {
  init_can_dbc_status = status;
}
bool matrix_sample::CanDecoder::GetCanDbcStatus() {
  return init_can_dbc_status;
}
void matrix_sample::CanDecoder::FeedInfo(int idx, int dlc, uint8_t data[8],
                                         int64_t ts) {
  struct can_frame can_frame_raw;
  can_frame_raw.can_id = idx;
  can_frame_raw.can_dlc = dlc;
  memcpy(can_frame_raw.data, data, 8);

  char buf[255];
  snprintf(buf, sizeof(buf), "CAN: id:%02x dlc:%d data:{ ",
           can_frame_raw.can_id, can_frame_raw.can_dlc);
  for (int i = 0; i < can_frame_raw.can_dlc; i++) {
    char buf2[100];
    snprintf(buf2, sizeof(buf2), "%02x ", can_frame_raw.data[i]);
    int len = strlen(buf);
    snprintf(buf + len, sizeof(buf) - len, "%s", buf2);
  }
  std::cout << buf << " }\n";

  HobotADAS::CAN::CANParser::Feed(&can_frame_raw, ts);
  const HobotADAS::CAN::CANFrameRaw &last_can_raw =
      HobotADAS::CAN::CANParser::GetLastCanFrameRaw();
  for (int i = 0; i < last_can_raw.info_count; i++) {
    if (last_can_raw.type[i] == HobotADAS::CAN::CAN_FRAME_TYPE_SPEED) {
      std::cout << "speed: " << last_can_raw.data[i].sp.speed << ", ";
    } else if (last_can_raw.type[i] ==
               HobotADAS::CAN::CAN_FRAME_TYPE_YAW_RATE) {
      std::cout << "yaw_rate: " << last_can_raw.data[i].yaw_rate.yaw_rate_
                << ", ";
    } else if (last_can_raw.type[i] ==
               HobotADAS::CAN::CAN_FRAME_TYPE_WHEEL_ANGLE) {
      std::cout << "wheel_angle: " << last_can_raw.data[i].wa.angle << ", ";
    } else if (last_can_raw.type[i] ==
               HobotADAS::CAN::CANRawDataType_VehicleSpeedValid) {
      std::cout << "VehicleSpeedValid: "
                << last_can_raw.data[i].VehicleSpeedValid.speed_valid << ", ";
    } else if (last_can_raw.type[i] ==
               HobotADAS::CAN::CAN_FRAME_TYPE_YAWRATEVALID) {
      std::cout << "YawRateValid: "
                << last_can_raw.data[i].YawRateValid.yawrate_valid << ", ";
    } else if (last_can_raw.type[i] == HobotADAS::CAN::CANRawDataType_Gear) {
      std::cout << "gear: " << last_can_raw.data[i].gt.type << ", ";
    } else if (last_can_raw.type[i] ==
               HobotADAS::CAN::CAN_FRAME_TYPE_TURN_SIGNAL) {
      std::cout << "turn_signal: " << last_can_raw.data[i].ts.type << ", ";
    } else if (last_can_raw.type[i] == HobotADAS::CAN::CAN_FRAME_TYPE_ACC) {
      std::cout << "acc: " << last_can_raw.data[i].acc.acc_ << ", ";
    }
  }
  std::cout << std::endl;
  {
    // consume
    HobotADAS::CAN::CANParser::can_list_.clear();
  }
}
