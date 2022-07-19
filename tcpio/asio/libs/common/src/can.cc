//
// Copyright 2016 Horizon Robotics.
//
#include "common/can.h"

#include <float.h>
#include <limits.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <list>
#include <string>
#include <utility>
#include <vector>

#include "common/timestamp.h"

#if defined(_WIN32)
#include <windows.h>
#elif defined(__ANDROID__)
//  include nothing now
#elif defined(__linux__) || defined(ADAS_FPGA)
// #include <spi_dbus_hal.h>
#include <dlfcn.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
// #include <can-utils/linux/can.h>
// #include <can-utils/linux/can/j1939.h>
#endif
typedef uint32_t uint32;
typedef int32_t int32;
typedef uint64_t uint64;
typedef uint16_t uint16;
namespace HobotADAS {
namespace CAN {
#if defined(LOW_GCC_VERSION)
#include <stdint.h>
#endif
CANFrame::CANFrame() {
  data_.resize(CAN_FRAME_TYPE_COUNT);
  // TODO(zhangyanle): set default values
#if 0
  int64_t ts = GetTimeStamp();
#else
  int64_t ts = 0;
#endif

  data_[CANRawDataType_Gear].gt.time_stamp_ = ts;
  data_[CANRawDataType_Gear].gt.type = GEAR_TYPE_P;

  data_[CANRawDataType_WheelAngle].wa.time_stamp_ = ts;
  data_[CANRawDataType_WheelAngle].wa.angle = 0.0f;

  data_[CANRawDataType_Speed].sp.time_stamp_ = ts;
  data_[CANRawDataType_Speed].sp.speed = FLT_EPSILON;

  data_[CANRawDataType_TurnSignal].ts.time_stamp_ = ts;
  data_[CANRawDataType_TurnSignal].ts.type = TURN_SIGNAL_NONE;

  data_[CANRawDataType_Acc].acc.time_stamp_ = ts;
  data_[CANRawDataType_Acc].acc.acc_ = 0.0f;

  data_[CANRawDataType_YawRate].yaw_rate.time_stamp_ = ts;
  data_[CANRawDataType_YawRate].yaw_rate.yaw_rate_ = 0.0f;

  data_[CANRawDataType_YawRateOffset].yaw_rate_offset.time_stamp_ = ts;
  data_[CANRawDataType_YawRateOffset].yaw_rate_offset.yaw_rate_offset_ = 0.0f;

  data_[CANRawDataType_lateral_acc].lateral_acc.time_stamp_ = ts;
  data_[CANRawDataType_lateral_acc].lateral_acc.lateral_acc_ = 0.0f;

  data_[CANRawDataType_brake_pedal_position].brake_pedal_position.time_stamp_ =
      ts;
  data_[CANRawDataType_brake_pedal_position]
      .brake_pedal_position.brake_pedal_position_ = 0.0f;

  data_[CANRawDataType_acc_pedal_position].acc_pedal_position.time_stamp_ = ts;
  data_[CANRawDataType_acc_pedal_position]
      .acc_pedal_position.acc_pedal_position_ = 0.0f;

  data_[CANRawDataType_power_mode].power_mode.time_stamp_ = ts;
  data_[CANRawDataType_power_mode].power_mode.power_mode_ = 0;

  data_[CANRawDataType_hazard_light].hazard_light.time_stamp_ = ts;
  data_[CANRawDataType_hazard_light].hazard_light.hazard_light_ = false;

  data_[CANRawDataType_head_wiper].head_wiper.time_stamp_ = ts;
  data_[CANRawDataType_head_wiper].head_wiper.head_wiper_ = 0;

  data_[CANRawDataType_outside_temperatrue].outside_temperatrue.time_stamp_ =
      ts;
  data_[CANRawDataType_outside_temperatrue]
      .outside_temperatrue.outside_temperatrue_ = 0;

  data_[CANRawDataType_light_level].light_level.time_stamp_ = ts;
  data_[CANRawDataType_light_level].light_level.light_level_ = 0;

  data_[CANRawDataType_hmw_enable].hmw_enable.time_stamp_ = ts;
  data_[CANRawDataType_hmw_enable].hmw_enable.hmw_enable_ = WarnEnable_ENABLE;

  data_[CANRawDataType_fcw_enable].fcw_enable.time_stamp_ = ts;
  data_[CANRawDataType_fcw_enable].fcw_enable.fcw_enable_ = WarnEnable_ENABLE;

  data_[CANRawDataType_ldw_enable].ldw_enable.time_stamp_ = ts;
  data_[CANRawDataType_ldw_enable].ldw_enable.ldw_enable_ = WarnEnable_ENABLE;

  data_[CANRawDataType_pcw_enable].pcw_enable.time_stamp_ = ts;
  data_[CANRawDataType_pcw_enable].pcw_enable.pcw_enable_ = WarnEnable_ENABLE;

  data_[CANRawDataType_ufcw_enable].ufcw_enable.time_stamp_ = ts;
  data_[CANRawDataType_ufcw_enable].ufcw_enable.ufcw_enable_ =
      WarnEnable_ENABLE;

  data_[CANRawDataType_ws_front_left].ws_front_left.time_stamp_ = ts;
  data_[CANRawDataType_ws_front_left].ws_front_left.ws_front_left_ =
      FLT_EPSILON;

  data_[CANRawDataType_ws_front_right].ws_front_right.time_stamp_ = ts;
  data_[CANRawDataType_ws_front_right].ws_front_right.ws_front_right_ =
      FLT_EPSILON;

  data_[CANRawDataType_ws_rear_left].ws_rear_left.time_stamp_ = ts;
  data_[CANRawDataType_ws_rear_left].ws_rear_left.ws_rear_left_ = FLT_EPSILON;

  data_[CANRawDataType_ws_rear_right].ws_rear_right.time_stamp_ = ts;
  data_[CANRawDataType_ws_rear_right].ws_rear_right.ws_rear_right_ = 0.0f;

  data_[CANRawDataType_ws_front_left_valid].ws_front_left_valid.time_stamp_ =
      ts;
  data_[CANRawDataType_ws_front_left_valid].ws_front_left_valid.ws_valid = true;

  data_[CANRawDataType_ws_front_right_valid].ws_front_right_valid.time_stamp_ =
      ts;
  data_[CANRawDataType_ws_front_right_valid].ws_front_right_valid.ws_valid =
      true;

  data_[CANRawDataType_ws_rear_left_valid].ws_rear_left_valid.time_stamp_ = ts;
  data_[CANRawDataType_ws_rear_left_valid].ws_rear_left_valid.ws_valid = true;

  data_[CANRawDataType_ws_rear_right_valid].ws_rear_right_valid.time_stamp_ =
      ts;
  data_[CANRawDataType_ws_rear_right_valid].ws_rear_right_valid.ws_valid = true;

  data_[CANRawDataType_ws_front_left_dir].ws_front_left_dir.time_stamp_ = ts;
  data_[CANRawDataType_ws_front_left_dir].ws_front_left_dir.ws_dir =
      WHEEL_DIR_TYPE_INVALID;

  data_[CANRawDataType_ws_front_right_dir].ws_front_right_dir.time_stamp_ = ts;
  data_[CANRawDataType_ws_front_right_dir].ws_front_right_dir.ws_dir =
      WHEEL_DIR_TYPE_INVALID;

  data_[CANRawDataType_ws_rear_left_dir].ws_rear_left_dir.time_stamp_ = ts;
  data_[CANRawDataType_ws_rear_left_dir].ws_rear_left_dir.ws_dir =
      WHEEL_DIR_TYPE_INVALID;

  data_[CANRawDataType_ws_rear_right_dir].ws_rear_right_dir.time_stamp_ = ts;
  data_[CANRawDataType_ws_rear_right_dir].ws_rear_right_dir.ws_dir =
      WHEEL_DIR_TYPE_INVALID;

  data_[CANRawDataType_WheelAngleValid].WheelAngleValid.time_stamp_ = ts;
  data_[CANRawDataType_WheelAngleValid].WheelAngleValid.wa_valid =
      CANValidType_VALID;

  data_[CANRawDataType_YawRateValid].YawRateValid.time_stamp_ = ts;
  data_[CANRawDataType_YawRateValid].YawRateValid.yawrate_valid =
      CANValidType_VALID;

  data_[CANRawDataType_WheelPulseFL].WheelPulseFL.time_stamp_ = ts;
  data_[CANRawDataType_WheelPulseFL].WheelPulseFL.whl_pule = 0;

  data_[CANRawDataType_WheelPulseFR].WheelPulseFR.time_stamp_ = ts;
  data_[CANRawDataType_WheelPulseFR].WheelPulseFR.whl_pule = 0;

  data_[CANRawDataType_WheelPulseRL].WheelPulseRL.time_stamp_ = ts;
  data_[CANRawDataType_WheelPulseRL].WheelPulseRL.whl_pule = 0;

  data_[CANRawDataType_WheelPulseRR].WheelPulseRR.time_stamp_ = ts;
  data_[CANRawDataType_WheelPulseRR].WheelPulseRR.whl_pule = 0;

  data_[CANRawDataType_WheelPulseFLValid].WheelPulseFLValid.time_stamp_ = ts;
  data_[CANRawDataType_WheelPulseFLValid].WheelPulseFLValid.whl_pule_valid =
      CANValidType_VALID;

  data_[CANRawDataType_WheelPulseFRValid].WheelPulseFRValid.time_stamp_ = ts;
  data_[CANRawDataType_WheelPulseFRValid].WheelPulseFRValid.whl_pule_valid =
      CANValidType_VALID;

  data_[CANRawDataType_WheelPulseRLValid].WheelPulseRLValid.time_stamp_ = ts;
  data_[CANRawDataType_WheelPulseRLValid].WheelPulseRLValid.whl_pule_valid =
      CANValidType_VALID;

  data_[CANRawDataType_WheelPulseRRValid].WheelPulseRRValid.time_stamp_ = ts;
  data_[CANRawDataType_WheelPulseRRValid].WheelPulseRRValid.whl_pule_valid =
      CANValidType_VALID;

  data_[CANRawDataType_WheelPulseFLDir].WheelPulseFLDir.time_stamp_ = ts;
  data_[CANRawDataType_WheelPulseFLDir].WheelPulseFLDir.whl_pule_dir =
      WHEEL_DIR_TYPE_INVALID;

  data_[CANRawDataType_WheelPulseFRDir].WheelPulseFRDir.time_stamp_ = ts;
  data_[CANRawDataType_WheelPulseFRDir].WheelPulseFRDir.whl_pule_dir =
      WHEEL_DIR_TYPE_INVALID;

  data_[CANRawDataType_WheelPulseRLDir].WheelPulseRLDir.time_stamp_ = ts;
  data_[CANRawDataType_WheelPulseRLDir].WheelPulseRLDir.whl_pule_dir =
      WHEEL_DIR_TYPE_INVALID;

  data_[CANRawDataType_WheelPulseRRDir].WheelPulseRRDir.time_stamp_ = ts;
  data_[CANRawDataType_WheelPulseRRDir].WheelPulseRRDir.whl_pule_dir =
      WHEEL_DIR_TYPE_INVALID;
  data_[CANRawDataType_seconds].CanTime_sec.tv_sec_ = 0;
  data_[CANRawDataType_seconds].CanTime_sec.time_stamp_ = 0;
  data_[CANRawDataType_nanoseconds].CanTime_nsec.tv_nsec_ = 0;
  data_[CANRawDataType_nanoseconds].CanTime_nsec.time_stamp_ = 0;
  data_[CANRawDataType_VehicleSpeedValid].VehicleSpeedValid.time_stamp_ = 0;
  data_[CANRawDataType_VehicleSpeedValid].VehicleSpeedValid.speed_valid =
      CANValidType_VALID;

  data_[CANRawDataType_IMU_Acc_X].imu_acc_x.time_stamp_ = ts;
  data_[CANRawDataType_IMU_Acc_X].imu_acc_x.imu_acc_x_ = 0;

  data_[CANRawDataType_IMU_Acc_Y].imu_acc_y.time_stamp_ = ts;
  data_[CANRawDataType_IMU_Acc_Y].imu_acc_y.imu_acc_y_ = 0;

  data_[CANRawDataType_IMU_Acc_Z].imu_acc_z.time_stamp_ = ts;
  data_[CANRawDataType_IMU_Acc_Z].imu_acc_z.imu_acc_z_ = 0;

  data_[CANRawDataType_IMU_Gyro_X].imu_gyro_x.time_stamp_ = ts;
  data_[CANRawDataType_IMU_Gyro_X].imu_gyro_x.imu_gyro_x_ = 0;

  data_[CANRawDataType_IMU_Gyro_Y].imu_gyro_y.time_stamp_ = ts;
  data_[CANRawDataType_IMU_Gyro_Y].imu_gyro_y.imu_gyro_y_ = 0;

  data_[CANRawDataType_IMU_Gyro_Z].imu_gyro_z.time_stamp_ = ts;
  data_[CANRawDataType_IMU_Gyro_Z].imu_gyro_z.imu_gyro_z_ = 0;

  data_[CANRawDataType_Yaw].yaw.time_stamp_ = ts;
  data_[CANRawDataType_Yaw].yaw.yaw_ = 0;

  data_[CANRawDataType_Odo_Pos_X].odo_pos_x.time_stamp_ = ts;
  data_[CANRawDataType_Odo_Pos_X].odo_pos_x.odo_pos_x_ = 0;

  data_[CANRawDataType_Odo_Pos_Y].odo_pos_y.time_stamp_ = ts;
  data_[CANRawDataType_Odo_Pos_Y].odo_pos_y.odo_pos_y_ = 0;

  data_[CANRawDataType_Odo_Pos_Z].odo_pos_z.time_stamp_ = ts;
  data_[CANRawDataType_Odo_Pos_Z].odo_pos_z.odo_pos_z_ = 0;

  data_[CANRawDataType_Odo_Speed].odo_speed.time_stamp_ = ts;
  data_[CANRawDataType_Odo_Speed].odo_speed.odo_speed_ = 0;

  data_[CANRawDataType_Odo_YawRate].odo_yawrate.time_stamp_ = ts;
  data_[CANRawDataType_Odo_YawRate].odo_yawrate.odo_yawrate_ = 0;

  data_[CANRawDataType_LeftTurnSignal].left_turn_signal.time_stamp_ = ts;
  data_[CANRawDataType_LeftTurnSignal].left_turn_signal.type = 0;

  data_[CANRawDataType_RightTurnSignal].right_turn_signal.time_stamp_ = ts;
  data_[CANRawDataType_RightTurnSignal].right_turn_signal.type = 0;

  data_[CANRawDataType_UDLCLeftStatus].udlc_left_status.time_stamp_ = ts;
  data_[CANRawDataType_UDLCLeftStatus].udlc_left_status.type = 0;

  data_[CANRawDataType_UDLCRightStatus].udlc_right_status.time_stamp_ = ts;
  data_[CANRawDataType_UDLCRightStatus].udlc_right_status.type = 0;
}

CANFrame::~CANFrame() { data_.clear(); }

static const char *can_raw_data_name_[CANRawDataType_COUNT] = {
    "gear",                   // CANRawDataType_Gear = 0,
    "wheel_angle",            // CANRawDataType_WheelAngle = 1,steering wheel
    "speed",                  // CANRawDataType_Speed = 2,
    "turn_signal",            // CANRawDataType_TurnSignal = 3,turning light
    "acc",                    // CANRawDataType_Acc = 4,longitudinal acc
    "yaw_rate",               // CANRawDataType_YawRate = 5,
    "yaw_rate_offset",        // CANRawDataType_YawRateOffset = 6
    "lateral_acc",            // CANRawDataType_lateral_acc = 7
    "brake_pedal_position",   // CANRawDataType_brake_pedal_position = 8,
    "acc_pedal_position",     // CANRawDataType_acc_pedal_position = 9,
    "power_mode",             // CANRawDataType_power_mode = 10,
    "hazard_light",           // CANRawDataType_hazard_light = 11,
    "head_wiper",             // CANRawDataType_head_wiper = 12,
    "outside_temperatrue",    // CANRawDataType_outside_temperatrue = 13,
    "light_level",            // CANRawDataType_light_level = 14,
    "hmw_enable",             // CANRawDataType_hmw_enable = 15,
    "fcw_enable",             // CANRawDataType_fcw_enable = 16,
    "ldw_enable",             // CANRawDataType_ldw_enable = 17,
    "pcw_enable",             // CANRawDataType_pcw_enable = 18,
    "ufcw_enable",            // CANRawDataType_ufcw_enable = 19,
    "ws_front_left",          // CAN_FRAME_TYPE_WS_FRONT_LEFT = 20,
    "ws_front_right",         // CAN_FRAME_TYPE_WS_FRONT_RIGHT = 21,
    "ws_rear_left",           // CAN_FRAME_TYPE_WS_REAR_LEFT = 22,
    "ws_rear_right",          // CAN_FRAME_TYPE_WS_REAR_RIGHT = 23,
    "yaw_rate_direction",     // CANRawDataType_YawRateDirection = 24,
    "wheel_angle_direction",  // CANRawDataType_WheelAngleDirection = 25,
    "ws_front_left_valid",    // CANRawDataType_ws_front_left_valid = 26,
    "ws_front_right_valid",   // CANRawDataType_ws_front_right_valid = 27,
    "ws_rear_left_valid",     // CANRawDataType_ws_rear_left_valid = 28,
    "ws_rear_right_valid",    // CANRawDataType_ws_rear_right_valid = 29,
    "ws_front_left_dir",      // CANRawDataType_ws_front_left_dir = 30,
    "ws_front_right_dir",     // CANRawDataType_ws_front_right_dir = 31,
    "ws_rear_left_dir",       // CANRawDataType_ws_rear_left_dir = 32,
    "ws_rear_right_dir",      // CANRawDataType_ws_rear_right_dir = 33,
    "WheelAngleValid",        // CANRawDataType_WheelAngleValid = 34,
    "YawRateValid",           // CANRawDataType_YawRateValid = 35,
    "WheelPulseFL",           // CANRawDataType_WheelPulseFL = 36,
    "WheelPulseFR",           // CANRawDataType_WheelPulseFR = 37,
    "WheelPulseRL",           // CANRawDataType_WheelPulseRL = 38,
    "WheelPulseRR",           // CANRawDataType_WheelPulseRR = 39,
    "WheelPulseFLDir",        // CANRawDataType_WheelPulseFLValid = 40,
    "WheelPulseFRDir",        // CANRawDataType_WheelPulseFRValid = 41,
    "WheelPulseRLDir",        // CANRawDataType_WheelPulseRLValid = 42,
    "WheelPulseRRDir",        // CANRawDataType_WheelPulseRRValid = 43,
    "WheelPulseFLValid",      // CANRawDataType_WheelPulseFLValid = 44,
    "WheelPulseFRValid",      // CANRawDataType_WheelPulseFRValid = 45,
    "WheelPulseRLValid",      // CANRawDataType_WheelPulseRLValid = 46,
    "WheelPulseRRValid",      // CANRawDataType_WheelPulseRRValid = 47,
    "seconds",                // CANRawDataType_seconds = 48,
    "nanoseconds",            // CANRawDataType_ nanoseconds_ = 49,
    "VehicleSpeedValid",      // CANRawDataType_ VehicleSpeedValid = 50
    "IMU_Acc_X",              // CANRawDataType_IMU_Acc_X = 51,
    "IMU_Acc_Y",              // CANRawDataType_IMU_Acc_Y = 52,
    "IMU_Acc_Z",              // CANRawDataType_IMU_Acc_Z = 53,
    "IMU_Gyro_X",             // CANRawDataType_IMU_Gyro_X = 54,
    "IMU_Gyro_Y",             // CANRawDataType_IMU_Gyro_Y = 55,
    "IMU_Gyro_Z",             // CANRawDataType_IMU_Gyro_Z = 56,
    "yaw",                    // CANRawDataType_Yaw = 57,
    "Odo_Pos_X",              // CANRawDataType_Odo_Pos_X = 58,
    "Odo_Pos_Y",              // CANRawDataType_Odo_Pos_Y = 59,
    "Odo_Pos_Z",              // CANRawDataType_Odo_Pos_Z = 60,
    "Odo_Speed",              // CANRawDataType_Odo_Speed = 61,
    "Odo_YawRate",            // CANRawDataType_Odo_YawRate = 62,
    "left_turn_signal",       // CANRawDataType_LeftTurnSignal = 63,
    "right_turn_signal",      // CANRawDataType_LeftTurnSignal = 64,
    "RoiStartX",              // CANRawDataType_RoiStartX = 65,
    "RoiStartY",              // CANRawDataType_RoiStartY = 66,
    "RoiWidth",               // CANRawDataType_RoiWidth = 67,
    "RoiHeight",              // CANRawDataType_RoiHeight = 68,
    "RoiStartXValid",         // CANRawDataType_RoiStartXValid = 69,
    "RoiStartYValid",         // CANRawDataType_RoiStartYValid = 70,
    "RoiWidthValid",          // CANRawDataType_RoiWidthValid = 71,
    "RoiHeightValid",         // CANRawDataType_RoiHeightValid = 72,
    "RoiRollingCount",        // CANRawDataType_RoiRollingCount = 73,
    "LowBeamStatus",          // CANRawDataType_LowBeamStatus = 74,
    "HighBeamStatus",         // CANRawDataType_HighBeamStatus = 75,
    "AutoHeadlightStatus",    // CANRawDataType_AutoHeadlightStatus = 76,
    "UDLCLeftStatus",         // CANRawDataType_UDLCLeftStatus = 77,
    "UDLCRightStatus"         // CANRawDataType_UDLCRightStatus = 78,
};

static uint64_t data_mask_[65];

static const char *GearTypeToString(int type) {
  if (type == GEAR_TYPE_P) {
    return "P";
  } else if (type == GEAR_TYPE_R) {
    return "R";
  } else if (type == GEAR_TYPE_N) {
    return "N";
  } else if (type == GEAR_TYPE_D) {
    return "D";
  } else {
    return "Invalid";
  }
}

static const char *TurnSignalTypeToString(int type) {
  if (type == TURN_SIGNAL_NONE) {
    return "None";
  } else if (type == TURN_SIGNAL_LEFT) {
    return "Left";
  } else if (type == TURN_SIGNAL_RIGHT) {
    return "Right";
  } else if (type == TURN_SIGNAL_DOUBLE) {
    return "Double";
  } else {
    return "Invalid";
  }
}

static const char *UDLCStatusTypeToString(int type) {
  if (type == UDLC_Not_Ready) {
    return "Not_Ready";
  } else if (type == UDLC_Ready) {
    return "Ready";
  } else if (type == UDLC_Active_Wait) {
    return "Active_Wait";
  } else if (type == UDLC_Active_Delay) {
    return "Active_Delay";
  } else if (type == UDLC_ChangeActive) {
    return "ChangeActive";
  } else if (type == UDLC_Not_Active_by_Obj) {
    return "Not_Active_by_Obj";
  } else {
    return "Invalid";
  }
}

static const char *PowerModeTypeToString(int type) {
  if (type == PowerMode_OFF) {
    return "OFF";
  } else if (type == PowerMode_ACC) {
    return "ACC";
  } else if (type == PowerMode_ON) {
    return "ON";
  } else if (type == PowerMode_CRANK) {
    return "CRANK";
  } else {
    return "Invalid";
  }
}

static const char *HazardLightTypeToString(int type) {
  if (type == HazardLight_OFF) {
    return "OFF";
  } else if (type == HazardLight_ON) {
    return "ON";
  } else {
    return "Invalid";
  }
}

static const char *HeadWiperTypeToString(int type) {
  if (type == HeadWiper_OFF) {
    return "OFF";
  } else if (type == HeadWiper_ACTIVE) {
    return "Active";
  } else if (type == HeadWiper_Reserved) {
    return "Reserved";
  } else {
    return "Invalid";
  }
}

static const char *LightLevelToString(int type) {
  if (type == LightLevel_UNKNOWN) {
    return "Unknown";
  } else if (type == LightLevel_NIGHT) {
    return "Night";
  } else if (type == LightLevel_DAY) {
    return "Day";
  } else {
    return "Invalid";
  }
}

static const char *WarnEnableTypeToString(int type) {
  if (type == WarnEnable_DISABLE) {
    return "Disable";
  } else if (type == WarnEnable_ENABLE) {
    return "Enable";
  } else if (type == WarnEnable_INHIBIT) {
    return "Inhibit";
  } else if (type == WarnEnable_RESERVED) {
    return "Reserved";
  } else {
    return "Invalid";
  }
}

void CANSetting::DescriptionToValueMap() {
  CANValueDescrip::iterator iter;
  switch (raw_type_) {
    case CANRawDataType_Gear: {
      for (iter = value_table_.begin(); iter != value_table_.end(); ++iter) {
        if (iter->second == "P") {
          value_Map_[iter->first] = GEAR_TYPE_P;
        } else if (iter->second == "R") {
          value_Map_[iter->first] = GEAR_TYPE_R;
        } else if (iter->second == "N") {
          value_Map_[iter->first] = GEAR_TYPE_N;
        } else if (iter->second == "D") {
          value_Map_[iter->first] = GEAR_TYPE_D;
        } else {
          value_Map_[iter->first] = GEAR_TYPE_NO_OP;
        }
      }  // end of for
    } break;
    case CANRawDataType_TurnSignal: {
      for (iter = value_table_.begin(); iter != value_table_.end(); ++iter) {
        if (iter->second == "Left") {
          value_Map_[iter->first] = TURN_SIGNAL_LEFT;
        } else if (iter->second == "Right") {
          value_Map_[iter->first] = TURN_SIGNAL_RIGHT;
        } else if (iter->second == "Double") {
          value_Map_[iter->first] = TURN_SIGNAL_DOUBLE;
        } else {
          value_Map_[iter->first] = TURN_SIGNAL_NONE;
        }
      }  // end of for
    } break;
    case CANRawDataType_UDLCLeftStatus:
    case CANRawDataType_UDLCRightStatus: {
      for (iter = value_table_.begin(); iter != value_table_.end(); ++iter) {
        if (iter->second == "Not_Ready") {
          value_Map_[iter->first] = UDLC_Not_Ready;
        } else if (iter->second == "Ready") {
          value_Map_[iter->first] = UDLC_Ready;
        } else if (iter->second == "Active_Wait") {
          value_Map_[iter->first] = UDLC_Active_Wait;
        } else if (iter->second == "Active_Delay") {
          value_Map_[iter->first] = UDLC_Active_Delay;
        } else if (iter->second == "ChangeActive") {
          value_Map_[iter->first] = UDLC_ChangeActive;
        } else if (iter->second == "Not_Active_by_Obj") {
          value_Map_[iter->first] = UDLC_Not_Active_by_Obj;
        } else {
          value_Map_[iter->first] = UDLC_Invalid;
        }
      }  // end of for
    } break;
    case CANRawDataType_LeftTurnSignal:
    case CANRawDataType_RightTurnSignal:
    case CANRawDataType_LowBeamStatus:
    case CANRawDataType_HighBeamStatus: {
      for (iter = value_table_.begin(); iter != value_table_.end(); ++iter) {
        if (iter->second == "Off") {
          value_Map_[iter->first] = LIGHT_OFF;
        } else if (iter->second == "On") {
          value_Map_[iter->first] = LIGHT_ON;
        } else {
          value_Map_[iter->first] = LIGHT_INVALID;
        }
      }  // end of for
    } break;
    case CANRawDataType_AutoHeadlightStatus: {
      for (iter = value_table_.begin(); iter != value_table_.end(); ++iter) {
        if (iter->second == "Active") {
          value_Map_[iter->first] = ACTIVE;
        } else if (iter->second == "Inactive") {
          value_Map_[iter->first] = INACTIVE;
        } else {
          value_Map_[iter->first] = INVALID;
        }
      }  // end of for
    } break;
    case CANRawDataType_power_mode: {
      for (iter = value_table_.begin(); iter != value_table_.end(); ++iter) {
        if (iter->second == "OFF") {
          value_Map_[iter->first] = PowerMode_OFF;
        } else if (iter->second == "ACC") {
          value_Map_[iter->first] = PowerMode_ACC;
        } else if (iter->second == "ON") {
          value_Map_[iter->first] = PowerMode_ON;
        } else if (iter->second == "CRANK") {
          value_Map_[iter->first] = PowerMode_CRANK;
        } else {
          value_Map_[iter->first] = PowerMode_OFF;
        }
      }  // end of for
    } break;
    case CANRawDataType_hazard_light: {
      for (iter = value_table_.begin(); iter != value_table_.end(); ++iter) {
        if (iter->second == "OFF") {
          value_Map_[iter->first] = HazardLight_OFF;
        } else if (iter->second == "ON") {
          value_Map_[iter->first] = HazardLight_ON;
        } else {
          value_Map_[iter->first] = HazardLight_OFF;
        }
      }  // end of for
    } break;
    case CANRawDataType_head_wiper: {
      for (iter = value_table_.begin(); iter != value_table_.end(); ++iter) {
        if (iter->second == "OFF") {
          value_Map_[iter->first] = HeadWiper_OFF;
        } else if (iter->second == "Active") {
          value_Map_[iter->first] = HeadWiper_ACTIVE;
        } else if (iter->second == "Reserved") {
          value_Map_[iter->first] = HeadWiper_Reserved;
        } else {
          value_Map_[iter->first] = HeadWiper_Reserved;
        }
      }  // end of for
    } break;
    case CANRawDataType_light_level: {
      for (iter = value_table_.begin(); iter != value_table_.end(); ++iter) {
        if (iter->second == "Unknown") {
          value_Map_[iter->first] = LightLevel_UNKNOWN;
        } else if (iter->second == "Night") {
          value_Map_[iter->first] = LightLevel_NIGHT;
        } else if (iter->second == "Day") {
          value_Map_[iter->first] = LightLevel_DAY;
        } else {
          value_Map_[iter->first] = LightLevel_UNKNOWN;
        }
      }  // end of for
    } break;
    case CANRawDataType_hmw_enable:
    case CANRawDataType_fcw_enable:
    case CANRawDataType_ldw_enable:
    case CANRawDataType_pcw_enable:
    case CANRawDataType_ufcw_enable: {
      for (iter = value_table_.begin(); iter != value_table_.end(); ++iter) {
        if (iter->second == "Disable") {
          value_Map_[iter->first] = WarnEnable_DISABLE;
        } else if (iter->second == "Enable") {
          value_Map_[iter->first] = WarnEnable_ENABLE;
        } else if (iter->second == "Inhibit") {
          value_Map_[iter->first] = WarnEnable_INHIBIT;
        } else if (iter->second == "Reserved") {
          value_Map_[iter->first] = WarnEnable_RESERVED;
        } else {
          value_Map_[iter->first] = WarnEnable_RESERVED;
        }
      }  // end of for
    } break;
    case CANRawDataType_YawRateDirection:
    case CANRawDataType_WheelAngleDirection: {
      for (iter = value_table_.begin(); iter != value_table_.end(); ++iter) {
        if (iter->second == "Negative") {
          value_Map_[iter->first] = DirectionType_Negative;
        } else if (iter->second == "Positive") {
          value_Map_[iter->first] = DirectionType_Positive;
        } else {
          value_Map_[iter->first] = DirectionType_Positive;
        }
      }  // end of forWarnEnable_RESERVED
    } break;

    case CANRawDataType_ws_front_left_valid:
    case CANRawDataType_ws_front_right_valid:
    case CANRawDataType_ws_rear_left_valid:
    case CANRawDataType_ws_rear_right_valid:
    case CANRawDataType_WheelAngleValid:
    case CANRawDataType_YawRateValid:
    case CANRawDataType_WheelPulseFLValid:
    case CANRawDataType_WheelPulseFRValid:
    case CANRawDataType_WheelPulseRLValid:
    case CANRawDataType_WheelPulseRRValid:
    case CANRawDataType_VehicleSpeedValid:
    case CANRawDataType_RoiStartXValid:
    case CANRawDataType_RoiStartYValid:
    case CANRawDataType_RoiWidthValid:
    case CANRawDataType_RoiHeightValid: {
      for (iter = value_table_.begin(); iter != value_table_.end(); ++iter) {
        if (iter->second == "Invalid") {
          value_Map_[iter->first] = CANValidType_INVALID;
        } else if (iter->second == "Valid") {
          value_Map_[iter->first] = CANValidType_VALID;
        } else {
          value_Map_[iter->first] = CANValidType_INVALID;
        }
      }  // end of for
    } break;
    case CANRawDataType_WheelPulseFLDir:
    case CANRawDataType_WheelPulseFRDir:
    case CANRawDataType_WheelPulseRLDir:
    case CANRawDataType_WheelPulseRRDir:
    case CANRawDataType_ws_front_left_dir:
    case CANRawDataType_ws_front_right_dir:
    case CANRawDataType_ws_rear_left_dir:
    case CANRawDataType_ws_rear_right_dir: {
      for (iter = value_table_.begin(); iter != value_table_.end(); ++iter) {
        if (iter->second == "Forward") {
          value_Map_[iter->first] = WHEEL_DIR_TYPE_FORWARD;
        } else if (iter->second == "Backward") {
          value_Map_[iter->first] = WHEEL_DIR_TYPE_BACKWARD;
        } else if (iter->second == "Standstill") {
          value_Map_[iter->first] = WHEEL_DIR_TYPE_STANDSTILL;
        } else if (iter->second == "Invalid") {
          value_Map_[iter->first] = WHEEL_DIR_TYPE_INVALID;
        } else {
          value_Map_[iter->first] = WHEEL_DIR_TYPE_INVALID;
        }
      }  // end of for
    } break;
    default: {
      break;
    }
  }  // end of switch
}

CANParser::CANParser() {}

CANParser::~CANParser() { Fini(); }

bool CANParser::Init(std::string config) {
  can_signal_timeout_config_.resize(CANRawDataType_COUNT);

  if (!GetCANProtocalSetting(config)) {
    return false;
  }
  //  std::cout << "get_can_protocal_setting success!" << std::endl;
  can_cvt_func_.resize(CANRawDataType_COUNT);
  can_cvt_func_[CANRawDataType_Gear] = &CANParser::ParseGear;
  can_cvt_func_[CANRawDataType_WheelAngle] = &CANParser::ParseWheelAngle;
  can_cvt_func_[CANRawDataType_Speed] = &CANParser::ParseVehSpeed;
  can_cvt_func_[CANRawDataType_TurnSignal] = &CANParser::ParseTurnSignal;
  can_cvt_func_[CANRawDataType_UDLCLeftStatus] =
      &CANParser::ParseUDLCLeftStatus;
  can_cvt_func_[CANRawDataType_UDLCRightStatus] =
      &CANParser::ParseUDLCRightStatus;
  can_cvt_func_[CANRawDataType_Acc] = &CANParser::ParseAcc;
  can_cvt_func_[CANRawDataType_YawRate] = &CANParser::ParseYawRate;
  can_cvt_func_[CANRawDataType_YawRateOffset] = NULL;
  can_cvt_func_[CANRawDataType_lateral_acc] = &CANParser::ParseLateralAcc;
  can_cvt_func_[CANRawDataType_brake_pedal_position] =
      &CANParser::ParseBrakePedalPosition;
  can_cvt_func_[CANRawDataType_acc_pedal_position] =
      &CANParser::ParseAccPedalPosition;
  can_cvt_func_[CANRawDataType_power_mode] = &CANParser::ParsePowerMode;
  can_cvt_func_[CANRawDataType_hazard_light] = &CANParser::ParseHazardLight;
  can_cvt_func_[CANRawDataType_head_wiper] = &CANParser::ParseHeadWiper;
  can_cvt_func_[CANRawDataType_outside_temperatrue] =
      &CANParser::ParseOutsideTemperatrue;
  can_cvt_func_[CANRawDataType_light_level] = &CANParser::ParseLightLevel;
  can_cvt_func_[CANRawDataType_hmw_enable] = &CANParser::ParseHMWEnable;
  can_cvt_func_[CANRawDataType_fcw_enable] = &CANParser::ParseFCWEnable;
  can_cvt_func_[CANRawDataType_ldw_enable] = &CANParser::ParseLateralAcc;
  can_cvt_func_[CANRawDataType_pcw_enable] = &CANParser::ParsePCWEnable;
  can_cvt_func_[CANRawDataType_ufcw_enable] = &CANParser::ParseUFCWEnable;
  can_cvt_func_[CANRawDataType_ws_front_left] = &CANParser::ParseWSFrontLeft;
  can_cvt_func_[CANRawDataType_ws_front_right] = &CANParser::ParseWSFrontRight;
  can_cvt_func_[CANRawDataType_ws_rear_left] = &CANParser::ParseWSRearLeft;
  can_cvt_func_[CANRawDataType_ws_rear_right] = &CANParser::ParseWSRearRight;
  can_cvt_func_[CANRawDataType_YawRateDirection] =
      &CANParser::ParseYawRateDirection;
  can_cvt_func_[CANRawDataType_WheelAngleDirection] =
      &CANParser::ParseWheelAngleDirection;
  can_cvt_func_[CANRawDataType_ws_front_left_valid] = NULL;
  can_cvt_func_[CANRawDataType_ws_front_right_valid] = NULL;
  can_cvt_func_[CANRawDataType_ws_rear_left_valid] = NULL;
  can_cvt_func_[CANRawDataType_ws_rear_right_valid] = NULL;
  can_cvt_func_[CANRawDataType_ws_front_left_dir] = NULL;
  can_cvt_func_[CANRawDataType_ws_front_right_dir] = NULL;
  can_cvt_func_[CANRawDataType_ws_rear_left_dir] = NULL;
  can_cvt_func_[CANRawDataType_ws_rear_right_dir] = NULL;
  can_cvt_func_[CANRawDataType_WheelAngleValid] = NULL;
  can_cvt_func_[CANRawDataType_YawRateValid] = &CANParser::ParseYawRateValid;
  can_cvt_func_[CANRawDataType_WheelPulseFL] = &CANParser::ParseWheelPulseFL;
  can_cvt_func_[CANRawDataType_WheelPulseFR] = &CANParser::ParseWheelPulseFR;
  can_cvt_func_[CANRawDataType_WheelPulseRL] = &CANParser::ParseWheelPulseRL;
  can_cvt_func_[CANRawDataType_WheelPulseRR] = &CANParser::ParseWheelPulseRR;
  can_cvt_func_[CANRawDataType_WheelPulseFLDir] =
      &CANParser::ParseWheelPulseFLDir;
  can_cvt_func_[CANRawDataType_WheelPulseFRDir] =
      &CANParser::ParseWheelPulseFRDir;
  can_cvt_func_[CANRawDataType_WheelPulseRLDir] =
      &CANParser::ParseWheelPulseRLDir;
  can_cvt_func_[CANRawDataType_WheelPulseRRDir] =
      &CANParser::ParseWheelPulseRRDir;
  can_cvt_func_[CANRawDataType_WheelPulseFLValid] = NULL;
  can_cvt_func_[CANRawDataType_WheelPulseFRValid] = NULL;
  can_cvt_func_[CANRawDataType_WheelPulseRLValid] = NULL;
  can_cvt_func_[CANRawDataType_WheelPulseRRValid] = NULL;
  can_cvt_func_[CANRawDataType_seconds] = &CANParser::Parseseconds;
  can_cvt_func_[CANRawDataType_nanoseconds] = &CANParser::Parsenanoseconds;
  can_cvt_func_[CANRawDataType_IMU_Acc_X] = &CANParser::ParseImuAccX;
  can_cvt_func_[CANRawDataType_IMU_Acc_Y] = &CANParser::ParseImuAccY;
  can_cvt_func_[CANRawDataType_IMU_Acc_Z] = &CANParser::ParseImuAccZ;
  can_cvt_func_[CANRawDataType_IMU_Gyro_X] = &CANParser::ParseImuGyroX;
  can_cvt_func_[CANRawDataType_IMU_Gyro_Y] = &CANParser::ParseImuGyroY;
  can_cvt_func_[CANRawDataType_IMU_Gyro_Z] = &CANParser::ParseImuGyroZ;
  can_cvt_func_[CANRawDataType_Yaw] = &CANParser::ParseCANYaw;
  can_cvt_func_[CANRawDataType_Odo_Pos_X] = &CANParser::ParseOdoPosX;
  can_cvt_func_[CANRawDataType_Odo_Pos_Y] = &CANParser::ParseOdoPosY;
  can_cvt_func_[CANRawDataType_Odo_Pos_Z] = &CANParser::ParseOdoPosZ;
  can_cvt_func_[CANRawDataType_Odo_Speed] = &CANParser::ParseOdoSpeed;
  can_cvt_func_[CANRawDataType_Odo_YawRate] = &CANParser::ParseOdoYawRate;
  can_cvt_func_[CANRawDataType_LeftTurnSignal] =
      &CANParser::ParseLeftTurnSignal;
  can_cvt_func_[CANRawDataType_RightTurnSignal] =
      &CANParser::ParseRightTurnSignal;
  can_cvt_func_[CANRawDataType_RoiStartX] = &CANParser::ParseRoiStartX;
  can_cvt_func_[CANRawDataType_RoiStartY] = &CANParser::ParseRoiStartY;
  can_cvt_func_[CANRawDataType_RoiWidth] = &CANParser::ParseRoiWidth;
  can_cvt_func_[CANRawDataType_RoiHeight] = &CANParser::ParseRoiHeight;
  can_cvt_func_[CANRawDataType_RoiRollingCount] =
      &CANParser::ParseRoiRollingCount;
  can_cvt_func_[CANRawDataType_LowBeamStatus] = &CANParser::ParseLowBeamStatus;
  can_cvt_func_[CANRawDataType_HighBeamStatus] =
      &CANParser::ParseHighBeamStatus;
  can_cvt_func_[CANRawDataType_AutoHeadlightStatus] =
      &CANParser::ParseAutoHeadlightStatus;

  can_filter_func_.resize(CANRawDataType_COUNT);
  can_filter_func_[CANRawDataType_Gear] = NULL;
  can_filter_func_[CANRawDataType_WheelAngle] = NULL;
  can_filter_func_[CANRawDataType_Speed] = &CANParser::FilterSpeed;
  can_filter_func_[CANRawDataType_TurnSignal] = &CANParser::FilterTurnSignal;
  can_filter_func_[CANRawDataType_Acc] = NULL;
  can_filter_func_[CANRawDataType_YawRate] = NULL;
  can_filter_func_[CANRawDataType_YawRateOffset] = NULL;
  can_filter_func_[CANRawDataType_lateral_acc] = NULL;
  can_filter_func_[CANRawDataType_brake_pedal_position] = NULL;
  can_filter_func_[CANRawDataType_acc_pedal_position] = NULL;
  can_filter_func_[CANRawDataType_power_mode] = NULL;
  can_filter_func_[CANRawDataType_hazard_light] = NULL;
  can_filter_func_[CANRawDataType_head_wiper] = NULL;
  can_filter_func_[CANRawDataType_outside_temperatrue] = NULL;
  can_filter_func_[CANRawDataType_light_level] = NULL;
  can_filter_func_[CANRawDataType_hmw_enable] = NULL;
  can_filter_func_[CANRawDataType_fcw_enable] = NULL;
  can_filter_func_[CANRawDataType_ldw_enable] = NULL;
  can_filter_func_[CANRawDataType_pcw_enable] = NULL;
  can_filter_func_[CANRawDataType_ufcw_enable] = NULL;
  can_filter_func_[CANRawDataType_ws_front_left] = NULL;
  can_filter_func_[CANRawDataType_ws_front_right] = NULL;
  can_filter_func_[CANRawDataType_ws_rear_left] = NULL;
  can_filter_func_[CANRawDataType_ws_rear_right] = NULL;
  can_filter_func_[CANRawDataType_YawRateDirection] = NULL;
  can_filter_func_[CANRawDataType_WheelAngleDirection] = NULL;
  can_filter_func_[CANRawDataType_ws_front_left_valid] = NULL;
  can_filter_func_[CANRawDataType_ws_front_right_valid] = NULL;
  can_filter_func_[CANRawDataType_ws_rear_left_valid] = NULL;
  can_filter_func_[CANRawDataType_ws_rear_right_valid] = NULL;
  can_filter_func_[CANRawDataType_WheelAngleValid] = NULL;
  can_filter_func_[CANRawDataType_YawRateValid] = NULL;
  can_filter_func_[CANRawDataType_WheelPulseFL] = NULL;
  can_filter_func_[CANRawDataType_WheelPulseFR] = NULL;
  can_filter_func_[CANRawDataType_WheelPulseRL] = NULL;
  can_filter_func_[CANRawDataType_WheelPulseRR] = NULL;
  can_filter_func_[CANRawDataType_WheelPulseFLDir] = NULL;
  can_filter_func_[CANRawDataType_WheelPulseFRDir] = NULL;
  can_filter_func_[CANRawDataType_WheelPulseRLDir] = NULL;
  can_filter_func_[CANRawDataType_WheelPulseRRDir] = NULL;
  can_filter_func_[CANRawDataType_WheelPulseFLValid] = NULL;
  can_filter_func_[CANRawDataType_WheelPulseFRValid] = NULL;
  can_filter_func_[CANRawDataType_WheelPulseRLValid] = NULL;
  can_filter_func_[CANRawDataType_WheelPulseRRValid] = NULL;
  can_filter_func_[CANRawDataType_ws_front_left_dir] = NULL;
  can_filter_func_[CANRawDataType_ws_front_right_dir] = NULL;
  can_filter_func_[CANRawDataType_ws_rear_left_dir] = NULL;
  can_filter_func_[CANRawDataType_ws_rear_right_dir] = NULL;
  can_filter_func_[CANRawDataType_seconds] = NULL;
  can_filter_func_[CANRawDataType_nanoseconds] = NULL;
  can_filter_func_[CANRawDataType_VehicleSpeedValid] = NULL;
  can_filter_func_[CANRawDataType_IMU_Acc_X] = NULL;
  can_filter_func_[CANRawDataType_IMU_Acc_Y] = NULL;
  can_filter_func_[CANRawDataType_IMU_Acc_Z] = NULL;
  can_filter_func_[CANRawDataType_IMU_Gyro_X] = NULL;
  can_filter_func_[CANRawDataType_IMU_Gyro_Y] = NULL;
  can_filter_func_[CANRawDataType_IMU_Gyro_Z] = NULL;
  can_filter_func_[CANRawDataType_Yaw] = NULL;
  can_filter_func_[CANRawDataType_Odo_Pos_X] = NULL;
  can_filter_func_[CANRawDataType_Odo_Pos_Y] = NULL;
  can_filter_func_[CANRawDataType_Odo_Pos_Z] = NULL;
  can_filter_func_[CANRawDataType_Odo_Speed] = NULL;
  can_filter_func_[CANRawDataType_Odo_YawRate] = NULL;
  can_filter_func_[CANRawDataType_LeftTurnSignal] = NULL;
  can_filter_func_[CANRawDataType_RightTurnSignal] = NULL;
  can_filter_func_[CANRawDataType_RoiStartX] = NULL;
  can_filter_func_[CANRawDataType_RoiStartY] = NULL;
  can_filter_func_[CANRawDataType_RoiWidth] = NULL;
  can_filter_func_[CANRawDataType_RoiHeight] = NULL;
  can_filter_func_[CANRawDataType_RoiStartXValid] = NULL;
  can_filter_func_[CANRawDataType_RoiStartYValid] = NULL;
  can_filter_func_[CANRawDataType_RoiWidthValid] = NULL;
  can_filter_func_[CANRawDataType_RoiHeightValid] = NULL;
  can_filter_func_[CANRawDataType_RoiRollingCount] = NULL;
  can_filter_func_[CANRawDataType_LowBeamStatus] = NULL;
  can_filter_func_[CANRawDataType_HighBeamStatus] = NULL;
  can_filter_func_[CANRawDataType_AutoHeadlightStatus] = NULL;
  can_filter_func_[CANRawDataType_UDLCLeftStatus] = NULL;
  can_filter_func_[CANRawDataType_UDLCRightStatus] = NULL;
  TimeStamp ts = GetTimeStamp();
  first_ts_status_ = true;

  last_raw_.resize(CAN_FRAME_TYPE_COUNT);
  last_raw_[CAN_FRAME_TYPE_GEAR].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_GEAR].type[0] = CAN_FRAME_TYPE_GEAR;
  last_raw_[CAN_FRAME_TYPE_GEAR].data[0].gt.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_GEAR].data[0].gt.type = GEAR_TYPE_P;
  last_raw_[CAN_FRAME_TYPE_GEAR].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_GEAR].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_GEAR].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_TURN_SIGNAL].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_TURN_SIGNAL].type[0] = CAN_FRAME_TYPE_TURN_SIGNAL;
  last_raw_[CAN_FRAME_TYPE_TURN_SIGNAL].data[0].ts.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_TURN_SIGNAL].data[0].ts.type = TURN_SIGNAL_NONE;
  last_raw_[CAN_FRAME_TYPE_TURN_SIGNAL].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_TURN_SIGNAL].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_TURN_SIGNAL].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_UDLCLeftStatus].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_UDLCLeftStatus].type[0] =
      CAN_FRAME_TYPE_UDLCLeftStatus;
  last_raw_[CAN_FRAME_TYPE_UDLCLeftStatus].data[0].ts.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_UDLCLeftStatus].data[0].ts.type = UDLC_Not_Ready;
  last_raw_[CAN_FRAME_TYPE_UDLCLeftStatus].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_UDLCLeftStatus].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_UDLCLeftStatus].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_UDLCRightStatus].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_UDLCRightStatus].type[0] =
      CAN_FRAME_TYPE_UDLCRightStatus;
  last_raw_[CAN_FRAME_TYPE_UDLCRightStatus].data[0].ts.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_UDLCRightStatus].data[0].ts.type = UDLC_Not_Ready;
  last_raw_[CAN_FRAME_TYPE_UDLCRightStatus].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_UDLCRightStatus].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_UDLCRightStatus].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_SPEED].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_SPEED].type[0] = CAN_FRAME_TYPE_SPEED;
  last_raw_[CAN_FRAME_TYPE_SPEED].data[0].sp.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_SPEED].data[0].sp.speed = FLT_EPSILON;
  last_raw_[CAN_FRAME_TYPE_SPEED].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_SPEED].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_SPEED].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEEL_ANGLE].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEEL_ANGLE].type[0] = CAN_FRAME_TYPE_WHEEL_ANGLE;
  last_raw_[CAN_FRAME_TYPE_WHEEL_ANGLE].data[0].wa.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEEL_ANGLE].data[0].wa.angle = 0.0f;
  last_raw_[CAN_FRAME_TYPE_WHEEL_ANGLE].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_WHEEL_ANGLE].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEEL_ANGLE].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_ACC].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_ACC].type[0] = CAN_FRAME_TYPE_ACC;
  last_raw_[CAN_FRAME_TYPE_ACC].data[0].acc.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_ACC].data[0].acc.acc_ = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_ACC].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_YAW_RATE].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_YAW_RATE].type[0] = CAN_FRAME_TYPE_YAW_RATE;
  last_raw_[CAN_FRAME_TYPE_YAW_RATE].data[0].yaw_rate.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_YAW_RATE].data[0].yaw_rate.yaw_rate_ = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_YAW_RATE].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_YAW_RATE_OFFSET].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_YAW_RATE_OFFSET].type[0] =
      CAN_FRAME_TYPE_YAW_RATE_OFFSET;
  last_raw_[CAN_FRAME_TYPE_YAW_RATE_OFFSET]
      .data[0]
      .yaw_rate_offset.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_YAW_RATE_OFFSET]
      .data[0]
      .yaw_rate_offset.yaw_rate_offset_ = 0.0f;
  memset(&last_raw_[CAN_FRAME_TYPE_YAW_RATE_OFFSET].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_LATERAL_ACC].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_LATERAL_ACC].type[0] = CAN_FRAME_TYPE_LATERAL_ACC;
  last_raw_[CAN_FRAME_TYPE_LATERAL_ACC].data[0].lateral_acc.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_LATERAL_ACC].data[0].lateral_acc.lateral_acc_ = 0.0f;
  memset(&last_raw_[CAN_FRAME_TYPE_LATERAL_ACC].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_BRAKE_PEDAL_POSITION].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_BRAKE_PEDAL_POSITION].type[0] =
      CAN_FRAME_TYPE_BRAKE_PEDAL_POSITION;
  last_raw_[CAN_FRAME_TYPE_BRAKE_PEDAL_POSITION]
      .data[0]
      .brake_pedal_position.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_BRAKE_PEDAL_POSITION]
      .data[0]
      .brake_pedal_position.brake_pedal_position_ = 0.0f;
  memset(&last_raw_[CAN_FRAME_TYPE_BRAKE_PEDAL_POSITION].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_ACC_PEDAL_POSITION].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_ACC_PEDAL_POSITION].type[0] =
      CAN_FRAME_TYPE_ACC_PEDAL_POSITION;
  last_raw_[CAN_FRAME_TYPE_ACC_PEDAL_POSITION]
      .data[0]
      .acc_pedal_position.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_ACC_PEDAL_POSITION]
      .data[0]
      .acc_pedal_position.acc_pedal_position_ = 0.0f;
  memset(&last_raw_[CAN_FRAME_TYPE_ACC_PEDAL_POSITION].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_POWER_MODE].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_POWER_MODE].type[0] = CAN_FRAME_TYPE_POWER_MODE;
  last_raw_[CAN_FRAME_TYPE_POWER_MODE].data[0].power_mode.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_POWER_MODE].data[0].power_mode.power_mode_ = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_POWER_MODE].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_HAZARD_LIGHT].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_HAZARD_LIGHT].type[0] = CAN_FRAME_TYPE_HAZARD_LIGHT;
  last_raw_[CAN_FRAME_TYPE_HAZARD_LIGHT].data[0].hazard_light.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_HAZARD_LIGHT].data[0].hazard_light.hazard_light_ =
      false;
  memset(&last_raw_[CAN_FRAME_TYPE_HAZARD_LIGHT].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_HEAD_WIPER].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_HEAD_WIPER].type[0] = CAN_FRAME_TYPE_HEAD_WIPER;
  last_raw_[CAN_FRAME_TYPE_HEAD_WIPER].data[0].head_wiper.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_HEAD_WIPER].data[0].head_wiper.head_wiper_ = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_HEAD_WIPER].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_OUTSIDE_TEMPERATRUE].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_OUTSIDE_TEMPERATRUE].type[0] =
      CAN_FRAME_TYPE_OUTSIDE_TEMPERATRUE;
  last_raw_[CAN_FRAME_TYPE_OUTSIDE_TEMPERATRUE]
      .data[0]
      .outside_temperatrue.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_OUTSIDE_TEMPERATRUE]
      .data[0]
      .outside_temperatrue.outside_temperatrue_ = 0.0f;
  memset(&last_raw_[CAN_FRAME_TYPE_OUTSIDE_TEMPERATRUE].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_LIGHT_LEVEL].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_LIGHT_LEVEL].type[0] = CAN_FRAME_TYPE_LIGHT_LEVEL;
  last_raw_[CAN_FRAME_TYPE_LIGHT_LEVEL].data[0].light_level.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_LIGHT_LEVEL].data[0].light_level.light_level_ = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_LIGHT_LEVEL].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_HMW_ENABLE].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_HMW_ENABLE].type[0] = CAN_FRAME_TYPE_HMW_ENABLE;
  last_raw_[CAN_FRAME_TYPE_HMW_ENABLE].data[0].hmw_enable.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_HMW_ENABLE].data[0].hmw_enable.hmw_enable_ =
      WarnEnable_ENABLE;
  memset(&last_raw_[CAN_FRAME_TYPE_HMW_ENABLE].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_FCW_ENABLE].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_FCW_ENABLE].type[0] = CAN_FRAME_TYPE_FCW_ENABLE;
  last_raw_[CAN_FRAME_TYPE_FCW_ENABLE].data[0].fcw_enable.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_FCW_ENABLE].data[0].fcw_enable.fcw_enable_ =
      WarnEnable_ENABLE;
  memset(&last_raw_[CAN_FRAME_TYPE_FCW_ENABLE].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_LDW_ENABLE].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_LDW_ENABLE].type[0] = CAN_FRAME_TYPE_LDW_ENABLE;
  last_raw_[CAN_FRAME_TYPE_LDW_ENABLE].data[0].ldw_enable.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_LDW_ENABLE].data[0].ldw_enable.ldw_enable_ =
      WarnEnable_ENABLE;
  memset(&last_raw_[CAN_FRAME_TYPE_LDW_ENABLE].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_PCW_ENABLE].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_PCW_ENABLE].type[0] = CAN_FRAME_TYPE_PCW_ENABLE;
  last_raw_[CAN_FRAME_TYPE_PCW_ENABLE].data[0].pcw_enable.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_PCW_ENABLE].data[0].pcw_enable.pcw_enable_ =
      WarnEnable_ENABLE;
  memset(&last_raw_[CAN_FRAME_TYPE_PCW_ENABLE].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_UFCW_ENABLE].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_UFCW_ENABLE].type[0] = CAN_FRAME_TYPE_UFCW_ENABLE;
  last_raw_[CAN_FRAME_TYPE_UFCW_ENABLE].data[0].ufcw_enable.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_UFCW_ENABLE].data[0].ufcw_enable.ufcw_enable_ =
      WarnEnable_ENABLE;
  memset(&last_raw_[CAN_FRAME_TYPE_UFCW_ENABLE].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WS_FRONT_LEFT].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_LEFT].type[0] =
      CAN_FRAME_TYPE_WS_FRONT_LEFT;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_LEFT].data[0].ws_front_left.time_stamp_ =
      ts;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_LEFT].data[0].ws_front_left.ws_front_left_ =
      FLT_EPSILON;
  memset(&last_raw_[CAN_FRAME_TYPE_WS_FRONT_LEFT].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT].type[0] =
      CAN_FRAME_TYPE_WS_FRONT_RIGHT;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT].data[0].ws_front_right.time_stamp_ =
      ts;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT]
      .data[0]
      .ws_front_right.ws_front_right_ = FLT_EPSILON;
  memset(&last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT].type[0] = CAN_FRAME_TYPE_WS_REAR_LEFT;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT].data[0].ws_rear_left.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT].data[0].ws_rear_left.ws_rear_left_ =
      FLT_EPSILON;
  memset(&last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT].type[0] =
      CAN_FRAME_TYPE_WS_REAR_RIGHT;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT].data[0].ws_rear_right.time_stamp_ =
      ts;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT].data[0].ws_rear_right.ws_rear_right_ =
      FLT_EPSILON;
  memset(&last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_YAW_RATE_DIRECTION].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_YAW_RATE_DIRECTION].type[0] =
      CAN_FRAME_TYPE_YAW_RATE_DIRECTION;
  last_raw_[CAN_FRAME_TYPE_YAW_RATE_DIRECTION]
      .data[0]
      .yaw_rate_direction.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_YAW_RATE_DIRECTION].data[0].yaw_rate_direction.type =
      DirectionType_Positive;
  memset(&last_raw_[CAN_FRAME_TYPE_YAW_RATE_DIRECTION].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEEL_ANGLE_DIRECTION].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEEL_ANGLE_DIRECTION].type[0] =
      CAN_FRAME_TYPE_WHEEL_ANGLE_DIRECTION;
  last_raw_[CAN_FRAME_TYPE_WHEEL_ANGLE_DIRECTION]
      .data[0]
      .wa_direction.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEEL_ANGLE_DIRECTION].data[0].wa_direction.type =
      DirectionType_Positive;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEEL_ANGLE_DIRECTION].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WS_FRONT_LEFT_VALID].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_LEFT_VALID].type[0] =
      CAN_FRAME_TYPE_WS_FRONT_LEFT_VALID;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_LEFT_VALID]
      .data[0]
      .ws_front_left_valid.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_LEFT_VALID]
      .data[0]
      .ws_front_left_valid.ws_valid = CANValidType_VALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WS_FRONT_LEFT_VALID].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT_VALID].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT_VALID].type[0] =
      CAN_FRAME_TYPE_WS_FRONT_RIGHT_VALID;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT_VALID]
      .data[0]
      .ws_front_right_valid.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT_VALID]
      .data[0]
      .ws_front_right_valid.ws_valid = CANValidType_VALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT_VALID].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT_VALID].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT_VALID].type[0] =
      CAN_FRAME_TYPE_WS_REAR_LEFT_VALID;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT_VALID]
      .data[0]
      .ws_rear_left_valid.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT_VALID]
      .data[0]
      .ws_rear_left_valid.ws_valid = CANValidType_VALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT_VALID].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT_VALID].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT_VALID].type[0] =
      CAN_FRAME_TYPE_WS_REAR_RIGHT_VALID;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT_VALID]
      .data[0]
      .ws_rear_right_valid.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT_VALID]
      .data[0]
      .ws_rear_right_valid.ws_valid = CANValidType_VALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT_VALID].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT_DIR].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT_DIR].type[0] =
      CAN_FRAME_TYPE_WS_FRONT_RIGHT_DIR;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT_DIR]
      .data[0]
      .ws_front_right_dir.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT_DIR]
      .data[0]
      .ws_front_right_dir.ws_dir = WHEEL_DIR_TYPE_INVALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WS_FRONT_RIGHT_DIR].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT_DIR].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT_DIR].type[0] =
      CAN_FRAME_TYPE_WS_REAR_LEFT_DIR;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT_DIR]
      .data[0]
      .ws_rear_left_dir.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT_DIR].data[0].ws_rear_left_dir.ws_dir =
      WHEEL_DIR_TYPE_INVALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WS_REAR_LEFT_DIR].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT_DIR].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT_DIR].type[0] =
      CAN_FRAME_TYPE_WS_REAR_RIGHT_DIR;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT_DIR]
      .data[0]
      .ws_rear_right_dir.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT_DIR].data[0].ws_rear_right_dir.ws_dir =
      WHEEL_DIR_TYPE_INVALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WS_REAR_RIGHT_DIR].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEELANGLEVALID].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEELANGLEVALID].type[0] =
      CAN_FRAME_TYPE_WHEELANGLEVALID;
  last_raw_[CAN_FRAME_TYPE_WHEELANGLEVALID]
      .data[0]
      .WheelAngleValid.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEELANGLEVALID].data[0].WheelAngleValid.wa_valid =
      CANValidType_VALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEELANGLEVALID].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_YAWRATEVALID].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_YAWRATEVALID].type[0] = CAN_FRAME_TYPE_YAWRATEVALID;
  last_raw_[CAN_FRAME_TYPE_YAWRATEVALID].data[0].WheelAngleValid.time_stamp_ =
      ts;
  last_raw_[CAN_FRAME_TYPE_YAWRATEVALID].data[0].WheelAngleValid.wa_valid =
      CANValidType_VALID;
  memset(&last_raw_[CAN_FRAME_TYPE_YAWRATEVALID].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFL].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFL].type[0] = CAN_FRAME_TYPE_WHEELPULSEFL;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFL].data[0].WheelPulseFL.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFL].data[0].WheelPulseFL.whl_pule = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEELPULSEFL].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFR].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFR].type[0] = CAN_FRAME_TYPE_WHEELPULSEFR;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFR].data[0].WheelPulseFR.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFR].data[0].WheelPulseFR.whl_pule = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEELPULSEFR].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEELPULSERL].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERL].type[0] = CAN_FRAME_TYPE_WHEELPULSERL;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERL].data[0].WheelPulseRL.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERL].data[0].WheelPulseRL.whl_pule = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEELPULSERL].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEELPULSERR].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERR].type[0] = CAN_FRAME_TYPE_WHEELPULSERR;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERR].data[0].WheelPulseRR.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERR].data[0].WheelPulseRR.whl_pule = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEELPULSERR].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFLDIR].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFLDIR].type[0] =
      CAN_FRAME_TYPE_WHEELPULSEFLDIR;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFLDIR]
      .data[0]
      .WheelPulseFLDir.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFLDIR]
      .data[0]
      .WheelPulseFLDir.whl_pule_dir = WHEEL_DIR_TYPE_INVALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEELPULSEFLDIR].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFRDIR].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFRDIR].type[0] =
      CAN_FRAME_TYPE_WHEELPULSEFRDIR;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFRDIR]
      .data[0]
      .WheelPulseFRDir.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFRDIR]
      .data[0]
      .WheelPulseFRDir.whl_pule_dir = WHEEL_DIR_TYPE_INVALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEELPULSEFRDIR].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEELPULSERLDIR].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERLDIR].type[0] =
      CAN_FRAME_TYPE_WHEELPULSERLDIR;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERLDIR]
      .data[0]
      .WheelPulseRLDir.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERLDIR]
      .data[0]
      .WheelPulseRLDir.whl_pule_dir = WHEEL_DIR_TYPE_INVALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEELPULSERLDIR].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEELPULSERRDIR].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERRDIR].type[0] =
      CAN_FRAME_TYPE_WHEELPULSERLDIR;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERRDIR]
      .data[0]
      .WheelPulseRRDir.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERRDIR]
      .data[0]
      .WheelPulseRRDir.whl_pule_dir = WHEEL_DIR_TYPE_INVALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEELPULSERRDIR].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFLVALID].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFLVALID].type[0] =
      CAN_FRAME_TYPE_WHEELPULSEFLVALID;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFLVALID]
      .data[0]
      .WheelPulseFLValid.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFLVALID]
      .data[0]
      .WheelPulseFLValid.whl_pule_valid = CANValidType_VALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEELPULSEFLVALID].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFRVALID].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFRVALID].type[0] =
      CAN_FRAME_TYPE_WHEELPULSEFRVALID;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFRVALID]
      .data[0]
      .WheelPulseFRValid.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSEFRVALID]
      .data[0]
      .WheelPulseFRValid.whl_pule_valid = CANValidType_VALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEELPULSEFRVALID].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEELPULSERLVALID].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERLVALID].type[0] =
      CAN_FRAME_TYPE_WHEELPULSERLVALID;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERLVALID]
      .data[0]
      .WheelPulseRLValid.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERLVALID]
      .data[0]
      .WheelPulseRLValid.whl_pule_valid = CANValidType_VALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEELPULSERLVALID].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_WHEELPULSERRVALID].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERRVALID].type[0] =
      CAN_FRAME_TYPE_WHEELPULSERRVALID;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERRVALID]
      .data[0]
      .WheelPulseRRValid.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_WHEELPULSERRVALID]
      .data[0]
      .WheelPulseRRValid.whl_pule_valid = CANValidType_VALID;
  memset(&last_raw_[CAN_FRAME_TYPE_WHEELPULSERRVALID].raw, 0,
         sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_IMU_ACC_X].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_X].type[0] = CAN_FRAME_TYPE_IMU_ACC_X;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_X].data[0].imu_acc_x.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_X].data[0].imu_acc_x.imu_acc_x_ = 0;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_X].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_X].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_IMU_ACC_X].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_IMU_ACC_Y].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_Y].type[0] = CAN_FRAME_TYPE_IMU_ACC_Y;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_Y].data[0].imu_acc_y.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_Y].data[0].imu_acc_y.imu_acc_y_ = 0;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_Y].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_Y].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_IMU_ACC_Y].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_IMU_ACC_Z].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_Z].type[0] = CAN_FRAME_TYPE_IMU_ACC_Z;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_Z].data[0].imu_acc_z.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_Z].data[0].imu_acc_z.imu_acc_z_ = 0;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_Z].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_IMU_ACC_Z].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_IMU_ACC_Z].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_X].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_X].type[0] = CAN_FRAME_TYPE_IMU_GYRO_X;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_X].data[0].imu_gyro_x.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_X].data[0].imu_gyro_x.imu_gyro_x_ = 0;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_X].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_X].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_IMU_GYRO_X].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Y].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Y].type[0] = CAN_FRAME_TYPE_IMU_GYRO_Y;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Y].data[0].imu_gyro_y.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Y].data[0].imu_gyro_y.imu_gyro_y_ = 0;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Y].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Y].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Y].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Z].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Z].type[0] = CAN_FRAME_TYPE_IMU_GYRO_Z;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Z].data[0].imu_gyro_z.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Z].data[0].imu_gyro_z.imu_gyro_z_ = 0;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Z].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Z].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_IMU_GYRO_Z].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_YAW].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_YAW].type[0] = CAN_FRAME_TYPE_YAW;
  last_raw_[CAN_FRAME_TYPE_YAW].data[0].yaw.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_YAW].data[0].yaw.yaw_ = 0;
  last_raw_[CAN_FRAME_TYPE_YAW].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_YAW].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_YAW].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_ODO_POS_X].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_X].type[0] = CAN_FRAME_TYPE_ODO_POS_X;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_X].data[0].odo_pos_x.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_X].data[0].odo_pos_x.odo_pos_x_ = 0;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_X].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_X].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_ODO_POS_X].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_ODO_POS_Y].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_Y].type[0] = CAN_FRAME_TYPE_ODO_POS_Y;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_Y].data[0].odo_pos_y.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_Y].data[0].odo_pos_y.odo_pos_y_ = 0;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_Y].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_Y].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_ODO_POS_Y].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_ODO_POS_Z].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_Z].type[0] = CAN_FRAME_TYPE_ODO_POS_Z;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_Z].data[0].odo_pos_z.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_Z].data[0].odo_pos_z.odo_pos_z_ = 0;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_Z].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_ODO_POS_Z].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_ODO_POS_Z].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_ODO_SPEED].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_ODO_SPEED].type[0] = CAN_FRAME_TYPE_ODO_SPEED;
  last_raw_[CAN_FRAME_TYPE_ODO_SPEED].data[0].odo_speed.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_ODO_SPEED].data[0].odo_speed.odo_speed_ = 0;
  last_raw_[CAN_FRAME_TYPE_ODO_SPEED].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_ODO_SPEED].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_ODO_SPEED].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_ODO_YAWRATE].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_ODO_YAWRATE].type[0] = CAN_FRAME_TYPE_ODO_YAWRATE;
  last_raw_[CAN_FRAME_TYPE_ODO_YAWRATE].data[0].odo_yawrate.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_ODO_YAWRATE].data[0].odo_yawrate.odo_yawrate_ = 0;
  last_raw_[CAN_FRAME_TYPE_ODO_YAWRATE].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_ODO_YAWRATE].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_ODO_YAWRATE].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_LEFT_TURN_SIGNAL].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_LEFT_TURN_SIGNAL].type[0] =
      CAN_FRAME_TYPE_LEFT_TURN_SIGNAL;
  last_raw_[CAN_FRAME_TYPE_LEFT_TURN_SIGNAL]
      .data[0]
      .left_turn_signal.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_LEFT_TURN_SIGNAL].data[0].left_turn_signal.type =
      LIGHT_OFF;
  last_raw_[CAN_FRAME_TYPE_LEFT_TURN_SIGNAL].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_LEFT_TURN_SIGNAL].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_LEFT_TURN_SIGNAL].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_RIGHT_TURN_SIGNAL].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_RIGHT_TURN_SIGNAL].type[0] =
      CAN_FRAME_TYPE_RIGHT_TURN_SIGNAL;
  last_raw_[CAN_FRAME_TYPE_RIGHT_TURN_SIGNAL]
      .data[0]
      .right_turn_signal.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_RIGHT_TURN_SIGNAL].data[0].right_turn_signal.type =
      LIGHT_OFF;
  last_raw_[CAN_FRAME_TYPE_RIGHT_TURN_SIGNAL].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_RIGHT_TURN_SIGNAL].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_RIGHT_TURN_SIGNAL].raw, 0,
         sizeof(can_frame));

#if 1
  last_raw_[CAN_FRAME_TYPE_seconds].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_seconds].type[0] = CAN_FRAME_TYPE_seconds;
  last_raw_[CAN_FRAME_TYPE_seconds].data[0].CanTime_sec.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_seconds].data[0].CanTime_sec.tv_sec_ = FLT_EPSILON;
  last_raw_[CAN_FRAME_TYPE_seconds].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_seconds].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_seconds].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_nanoseconds].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_nanoseconds].type[0] = CAN_FRAME_TYPE_nanoseconds;
  last_raw_[CAN_FRAME_TYPE_nanoseconds].data[0].CanTime_sec.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_nanoseconds].data[0].CanTime_sec.tv_sec_ =
      FLT_EPSILON;
  last_raw_[CAN_FRAME_TYPE_nanoseconds].raw.can_id = 0;
  last_raw_[CAN_FRAME_TYPE_nanoseconds].raw.can_dlc = 0;
  memset(&last_raw_[CAN_FRAME_TYPE_nanoseconds].raw, 0, sizeof(can_frame));

  last_raw_[CAN_FRAME_TYPE_VehicleSpeedValid].info_count = 1;
  last_raw_[CAN_FRAME_TYPE_VehicleSpeedValid].type[0] =
      CAN_FRAME_TYPE_VehicleSpeedValid;
  last_raw_[CAN_FRAME_TYPE_VehicleSpeedValid]
      .data[0]
      .VehicleSpeedValid.time_stamp_ = ts;
  last_raw_[CAN_FRAME_TYPE_VehicleSpeedValid]
      .data[0]
      .VehicleSpeedValid.speed_valid = CANValidType_VALID;
  memset(&last_raw_[CAN_FRAME_TYPE_VehicleSpeedValid].raw, 0,
         sizeof(can_frame));

#endif
  last_ts_status_ = last_raw_[CAN_FRAME_TYPE_TURN_SIGNAL].data->ts;
  if (data_mask_[1] == 0) {
    for (int i = 0; i < 64; i++) {
      data_mask_[i] = (1LLU << i) - 1;
    }
    data_mask_[64] = UINT64_MAX;
  }

  can_signal_status_.resize(CANRawDataType_COUNT);
  for (int i = 0; i < CANRawDataType_COUNT; i++) {
    can_signal_status_[i].timeout_flag_ = false;
    can_signal_status_[i].last_timestamp_ = ts;
  }
  can_diag_state_.resize(CANSignal_Diagnosis_Num);
  for (int i = 0; i < CANSignal_Diagnosis_Num; i++) {
    can_diag_state_[i].pre_ = CANSignal_Diagnosis_Status_Init;
    can_diag_state_[i].cur_ = CANSignal_Diagnosis_Status_Init;
  }
  check_signal_can_.resize(CANRawDataType_COUNT);
  for (int i = 0; i < CANRawDataType_COUNT; i++) {
    check_signal_can_[i] = false;
  }
  return true;
}  // namespace CAN

void CANParser::Fini() {
  {
    std::lock_guard<std::mutex> lck(can_mutex_);
    can_list_.clear();
  }
  {
    std::lock_guard<std::mutex> lckfd(canfd_mutex_);
    canfd_list_.clear();
  }
  last_raw_.clear();
  can_cvt_func_.clear();
  can_filter_func_.clear();
  frame_map_.clear();
  can_signal_status_.clear();
  can_signal_timeout_config_.clear();
  can_diag_state_.clear();
}

bool CANParser::GetCANProtocalSetting(std::string path) {
  try {
    std::ifstream cfg_file(path);
    const auto &cfg = nlohmann::json::parse(cfg_file);

    turn_signal_interval_ = cfg.value("turn_signal_interval", 300);
    // use it to logging all can data
    auto verbose = cfg.value("verbose", 0);
    if (verbose != 0) {
      verbose_mode_ = true;
    }
    verbose_mode_ = true;
    max_can_frame_ = cfg.value("max_can_frame", 50);
    prefetch_time_ = cfg.value("prefetch_time", 34);

    int idx_num = 0;
    for (auto &&dbc : cfg.at("dbc")) {
      idx_num++;
      auto can_id = dbc.value("id", "");
      if (can_id == "") {
        continue;
      }
      int canID = strtoll(can_id.c_str(), NULL, 16);
      if (frame_map_.find(canID) == frame_map_.end()) {
        std::vector<CANSetting> settings;
        frame_map_.insert(std::make_pair(canID, settings));
      }
      std::vector<CANSetting> &setting_vec = frame_map_.at(canID);
      auto str_endian = dbc.value("endian", "Intel");

      // auto can_data_config = dbc.at("data");
      int data_count = 0;
      for (auto data_config : dbc.at("data")) {
        data_count++;
        CANSetting setting;
        int endian = str_endian == "Motorola" ? CANRawEndianType_Big
                                              : CANRawEndianType_Little;
        setting.can_id_ = canID;
        setting.endian_ = endian;
        auto data_name = data_config.value("name", "");
        auto parser_flag = data_config.value("parser_flag", 1);
        //     std::cout << "data_name:" << data_name << "  parser_flag:" <<
        //     parser_flag << std::endl;
        for (int k = 0; k < CANRawDataType_COUNT; k++) {
          if (data_name == can_raw_data_name_[k]) {
            if (parser_flag || verbose_mode_) {
              setting.raw_type_ = k;
            }
            break;
          }
        }
        if ((setting.raw_type_ >= 0) &&
            (setting.raw_type_ < CANRawDataType_COUNT)) {
          setting.start_bit_ = data_config.value("start_bit", 0);
          setting.bit_length_ = data_config.value("bit_length", 0);
          if (endian == CANRawEndianType_Big) {
            int byte_num = setting.start_bit_ / 8;
            int bit_in_byte = (setting.start_bit_ % 8);
            setting.start_bit_ = 64 - (byte_num + 1) * 8 + bit_in_byte;
          }

          std::string str_type = data_config.value("value_type", "Unsigned");
          if (str_type == "Signed") {
            setting.value_type_ = CANRawDataValue_Signed;
          } else if (str_type == "Float") {
            setting.value_type_ = CANRawDataValue_Float;
          } else if (str_type == "Double") {
            setting.value_type_ = CANRawDataValue_Double;
          } else if (str_type == "Enum") {
            setting.value_type_ = CANRawDataValue_Enum;
          } else {
            setting.value_type_ = CANRawDataValue_Unsigned;
          }
          setting.scale_ = data_config.value("scale", 1.0f);
          setting.offset_ = data_config.value("offset", 0.0f);
          auto range_config = data_config.at("range");
          setting.min_ = range_config.value("min", 0.0f);
          setting.max_ = range_config.value("max", 0.0f);
          if (setting.value_type_ == CANRawDataValue_Enum) {
            // auto mapping_config = data_config.at("value_table");
            int value_num = 0;
            for (auto value_table : data_config.at("value_table")) {
              value_num++;
              std::string val_str = value_table.value("value", "");
              int32_t val =
                  static_cast<int32_t>(strtoul(val_str.c_str(), NULL, 0));
              std::string description = value_table.value("desp", "");
              setting.value_table_[val] = description;
            }
            if (value_num == 0) {
              return false;
            }
            setting.DescriptionToValueMap();
            if (setting.value_Map_.empty()) {
              // LOG(WARNING) << "No Mathing Values for "
              //<< std::hex << setting.can_id_;
            }
          }
          setting_vec.push_back(setting);

          can_signal_timeout_config_[setting.raw_type_].timeout_time_ =
              data_config.value("timeout_time", 0);  // ms
          if (can_signal_timeout_config_[setting.raw_type_].timeout_time_ > 0) {
            can_signal_timeout_config_[setting.raw_type_].timeout_monitor_ =
                true;
          } else {
            can_signal_timeout_config_[setting.raw_type_].timeout_monitor_ =
                false;
          }
        }  //  end of if raw data type
      }
    }
  } catch (const nlohmann::json::parse_error &e) {
    std::cerr << "parse failed-parse_error" << e.what()
              << std::endl;
    return false;
  } catch (const nlohmann::json::out_of_range &e) {
    std::cerr << "parse failed-out_of_range" << e.what()
              << std::endl;
    return false;
  }
  return true;
}

bool CANParser::AddCanRaw(struct can_frame *frame, TimeStamp ts) {
  {
    std::lock_guard<std::mutex> lck(can_mutex_);
    CANFrameRaw raw;
    memset(&raw, 0x0, sizeof(raw));
    raw.raw = *frame;
    raw.raw_ts = ts;
    can_list_.push_back(raw);
    if (can_list_.size() > max_can_frame_) {
      can_list_.pop_front();
    }
  }
  return true;
}

bool CANParser::Feed(struct can_frame *frame, TimeStamp ts) {
  CANFrameRaw raw;
  memset(&raw, 0x0, sizeof(raw));
  int valid_raw = ParseCanFrame(ts, frame, &raw);
  if (verbose_mode_ && valid_raw == 0) {
    valid_raw = 1;
    raw.info_count = 1;
    raw.type[0] = CAN_FRAME_TYPE_EXTRA;
    raw.raw = *frame;
    raw.raw_ts = ts;
  }

  if (valid_raw) {
    std::lock_guard<std::mutex> lck(can_mutex_);
    for (int i = 0; i < valid_raw; i++) {
      if (raw.type[i] != CAN_FRAME_TYPE_EXTRA) {
        last_raw_[raw.type[i]] = raw;
        last_raw_[raw.type[i]].info_count = 1;
        last_raw_[raw.type[i]].type[0] = raw.type[i];
        last_raw_[raw.type[i]].data[0] = raw.data[i];
      }
    }
    can_list_.push_back(raw);
    if (can_list_.size() > max_can_frame_) {
      can_list_.pop_front();
    }
    last_can_frame_raw_ = raw;
    return true;
  } else {
    last_can_frame_raw_.info_count = 0;
    return false;
  }
}

bool CANParser::Feed(struct canfd_frame *frame, TimeStamp ts) {
  {
    std::lock_guard<std::mutex> lck(canfd_mutex_);
    CANFDFrameRaw raw;
    memset(&raw, 0x0, sizeof(raw));
    raw.raw = *frame;
    raw.raw_ts = ts;
    canfd_list_.push_back(raw);
    if (canfd_list_.size() > max_can_frame_) {
      canfd_list_.pop_front();
    }
  }
  return true;
}

bool CANParser::GetCANFilterSetting(std::string path,
                                    std::vector<struct CANFilter_st> &filter) {
  std::ifstream cfg_file(path);

  const auto &cfg = nlohmann::json::parse(cfg_file);

  turn_signal_interval_ = cfg.value("turn_signal_interval", 300);
  int idx_num = 0;
  for (auto can_config : cfg.at("dbc")) {
    idx_num++;
    struct CANFilter_st can_filter;
    uint8_t signal_num = 0;
    can_filter.signals_.clear();
    int can_id = can_config.value("id", -1);
    if (can_id == -1) {
      std::string str_id = can_config.value("id", "");
      if (str_id.empty()) {
        continue;
      }
      can_id = static_cast<int>(strtoul(str_id.c_str(), NULL, 0));
    }
    can_filter.can_id_ = can_id;
    std::string str_endian = can_config.value("endian", "Intel");
    int endian = str_endian == "Motorola" ? CANRawEndianType_Big
                                          : CANRawEndianType_Little;
    can_filter.endian_ = static_cast<uint8_t>(endian);
    uint16 cycle = static_cast<uint16>(can_config.value("cycle", 0));
    can_filter.cycle_ = cycle;
    int data_count = 0;
    for (auto data_config : can_config.at("data")) {
      data_count++;
      CANSignal_st can_signal;
      std::string data_name = data_config.value("name", "");
      for (int k = 0; k < CANRawDataType_COUNT; k++) {
        if (data_name == can_raw_data_name_[k]) {
          can_signal.singal_ = static_cast<uint8_t>(k);
          can_signal.start_bit_ =
              static_cast<uint8_t>(data_config.value("start_bit", 0));
          can_signal.bit_length_ =
              static_cast<uint8_t>(data_config.value("bit_length", 0));
          std::string str_type = data_config.value("value_type", "Unsigned");
          uint8_t value_type;
          if (str_type == "Signed") {
            value_type = CANRawDataValue_Signed;
          } else if (str_type == "Float") {
            value_type = CANRawDataValue_Float;
          } else if (str_type == "Double") {
            value_type = CANRawDataValue_Double;
          } else if (str_type == "Enum") {
            value_type = CANRawDataValue_Enum;
          } else {
            value_type = CANRawDataValue_Unsigned;
          }
          can_signal.value_type_ = value_type;

          can_signal.scale_ = data_config.value("scale", 1.0f);
          can_signal.offset_ = data_config.value("offset", 0.0f);
          can_signal.map_size_ = 0;
          if (can_signal.value_type_ == CANRawDataValue_Enum) {
            CANSetting setting;
            setting.raw_type_ = can_signal.singal_;
            int value_num = 0;
            for (auto mapping_config : data_config.at("value_table")) {
              value_num++;
              std::string val_str = mapping_config.value("value", "");
              int32_t val =
                  static_cast<int32_t>(strtoul(val_str.c_str(), NULL, 0));

              std::string description = mapping_config.value("desp", "");
              setting.value_table_[val] = description;
            }
            if (value_num == 0) {
              // LOG(ERROR) << "enum value table";
              return false;
            }
            setting.DescriptionToValueMap();
            if (setting.value_Map_.empty()) {
              // LOG(WARNING) << "No Mathing Values for "
              //<< std::hex << setting.can_id_;
            }

            switch (can_signal.singal_) {
              case CANRawDataType_Gear: {
                can_signal.value_map_.resize(16, GEAR_TYPE_NO_OP);
                can_signal.map_size_ = 16;
                break;
              }
              case CANRawDataType_TurnSignal: {
                can_signal.value_map_.resize(8, TURN_SIGNAL_NONE);
                can_signal.map_size_ = 8;
                break;
              }
              case CANRawDataType_UDLCLeftStatus:
              case CANRawDataType_UDLCRightStatus: {
                can_signal.value_map_.resize(16, UDLC_Not_Ready);
                can_signal.map_size_ = 8;
                break;
              }
              case CANRawDataType_WheelAngleValid:
              case CANRawDataType_YawRateValid:
              case CANRawDataType_VehicleSpeedValid: {
                can_signal.value_map_.resize(8, CANValidType_INVALID);
                can_signal.map_size_ = 8;
                break;
              }
              default: {
                can_signal.value_map_.clear();
                can_signal.map_size_ = 0;
                break;
              }
            }
            for (int m = 0; m < setting.value_Map_.size(); m++) {
              if (m < can_signal.value_map_.size()) {
                can_signal.value_map_[m] =
                    static_cast<uint8_t>(setting.value_Map_[m]);
              }
            }
          }
          can_filter.signals_.push_back(can_signal);
          signal_num++;
          break;
        }
      }
      if (can_filter.signals_.size() > 0) {
        can_filter.signal_num_ = signal_num;
        filter.push_back(can_filter);
      }
    }
    if (idx_num == 0) {
      return false;
    }

    return true;
  }
}

bool CANParser::Summarize(TimeStamp time_stamp, CANFrame &can_frame,
                          std::vector<CAN::CANFrameRaw> &can_frame_raw,
                          std::vector<CAN::CANFDFrameRaw> &canfd_frame_raw) {
  {
    std::lock_guard<std::mutex> lck(can_mutex_);
    if (!can_list_.empty()) {
      int64_t target_ts = time_stamp;
      while (!can_list_.empty()) {
        const CANFrameRaw &raw = can_list_.front();

        // if (can_list_.size() < 50
        //   && raw.raw_ts > target_ts) {
        if (raw.raw_ts > target_ts) {
          break;
        }
        can_frame_raw.push_back(raw);
        can_list_.pop_front();
      }

      if (can_frame_raw.empty()) {
        while (can_list_.size() > max_can_frame_) {
          can_list_.pop_front();
        }
      }

      target_ts += prefetch_time_;
      std::list<CANFrameRaw>::iterator iter;
      for (iter = can_list_.begin(); iter != can_list_.end(); ++iter) {
        if (iter->raw_ts < target_ts) {
          can_frame_raw.push_back(*iter);
        }  // end of if
      }    // end of for iter
    }      // end of if can list
  }
  {
    std::lock_guard<std::mutex> lckfd(canfd_mutex_);
    if (!canfd_list_.empty()) {
      int64_t target_ts = time_stamp;
      while (!canfd_list_.empty()) {
        const CANFDFrameRaw &raw = canfd_list_.front();
        // if (canfd_list_.size() < 50
        //   && raw.raw_ts > target_ts) {
        if (raw.raw_ts > target_ts) {
          break;
        }
        canfd_frame_raw.push_back(raw);
        canfd_list_.pop_front();
      }

      if (canfd_frame_raw.empty()) {
        while (canfd_list_.size() > max_can_frame_) {
          canfd_list_.pop_front();
        }
      }
      target_ts += prefetch_time_;
      std::list<CANFDFrameRaw>::iterator iter;
      for (iter = canfd_list_.begin(); iter != canfd_list_.end(); ++iter) {
        if (iter->raw_ts < target_ts) {
          canfd_frame_raw.push_back(*iter);
        }  // end of if
      }    // end of for iter
    }      // end of if canfd list
  }
#if 0
  bool have_gear = false;
  bool have_speed = false;
  bool have_wheel_angle = false;
  bool have_turn_signal = false;
  bool have_acc = false;
  bool have_yaw_rate = false;
  for (size_t i = 0; i < can_frame_raw.size(); i++) {
    const CANFrameRaw &raw = can_frame_raw[i];
    for (int j = 0; j < raw.info_count; j++) {
      if (raw.type[j] == CAN_FRAME_TYPE_GEAR) {
        have_gear = true;
      } else if (raw.type[j] == CAN_FRAME_TYPE_SPEED) {
        have_speed = true;
      }
      if (raw.type[j] == CAN_FRAME_TYPE_WHEEL_ANGLE) {
        have_wheel_angle = true;
      }
      if (raw.type[j] == CAN_FRAME_TYPE_TURN_SIGNAL) {
        have_turn_signal = true;
      }
      if (raw.type[j] == CAN_FRAME_TYPE_ACC) {
        have_acc = true;
      }
      if (raw.type[j] == CAN_FRAME_TYPE_YAW_RATE) {
        have_yaw_rate = true;
      }
    }
  }

  if (!have_gear) {
    can_frame_raw.push_back(last_raw_[CAN_FRAME_TYPE_GEAR]);
  }
  if (!have_speed) {
    can_frame_raw.push_back(last_raw_[CAN_FRAME_TYPE_SPEED]);
  }
  if (!have_wheel_angle) {
    can_frame_raw.push_back(last_raw_[CAN_FRAME_TYPE_WHEEL_ANGLE]);
  }
  if (!have_turn_signal) {
    can_frame_raw.push_back(last_raw_[CAN_FRAME_TYPE_TURN_SIGNAL]);
  }
  if (!have_acc) {
    can_frame_raw.push_back(last_raw_[CAN_FRAME_TYPE_ACC]);
  }
  if (!have_yaw_rate) {
    can_frame_raw.push_back(last_raw_[CAN_FRAME_TYPE_YAW_RATE]);
  }
#else
  std::vector<bool> have_raw_data(CAN_FRAME_TYPE_PROCESS_COUNT, false);
  int64_t current_ts = time_stamp;
  for (size_t i = 0; i < can_frame_raw.size(); i++) {
    const CANFrameRaw &raw = can_frame_raw[i];
    for (int j = 0; j < raw.info_count; j++) {
      if ((raw.type[j] < CAN_FRAME_TYPE_PROCESS_COUNT) && (raw.type[j] >= 0)) {
        have_raw_data[raw.type[j]] = true;
        can_signal_status_[raw.type[j]].last_timestamp_ = current_ts;
        check_signal_can_[raw.type[j]] = true;
        can_signal_status_[raw.type[j]].timeout_flag_ = false;
        if (raw.type[j] == CAN_FRAME_TYPE_SPEED) {
          can_diag_state_[CANSignal_Diagnosis_Speed_Timeout].cur_ =
              CANSignal_Diagnosis_Status_Pass;
        }
        if (raw.type[j] == CAN_FRAME_TYPE_YAW_RATE) {
          can_diag_state_[CANSignal_Diagnosis_Yawrate_Timeout].cur_ =
              CANSignal_Diagnosis_Status_Pass;
        }
      }
    }  // end of if
  }    // end of for j
       // check if don't have the new raw data, use the last raw value

  // VLOG(0) << "last_speed_time" <<
  //         can_signal_status_[CAN_FRAME_TYPE_SPEED].last_timestamp_
  //         << "current_speed_time" <<current_ts;
  int64_t compare_time;
  for (int k = 0; k < CAN_FRAME_TYPE_PROCESS_COUNT; k++) {
    if (!have_raw_data[k]) {
      // check message time out
      if (check_signal_can_[k]) {
        compare_time = can_signal_timeout_config_[k].timeout_time_;
      } else {
        compare_time = 10000;
      }
      if (can_signal_timeout_config_[k].timeout_monitor_ &&
          !can_signal_status_[k].timeout_flag_ &&
#ifdef __QNX__
          std::llabs(can_signal_status_[k].last_timestamp_ - current_ts) >
#else
          llabs(can_signal_status_[k].last_timestamp_ - current_ts) >
#endif
              compare_time) {
        can_signal_status_[k].timeout_flag_ = true;
      }
      if (k == CAN_FRAME_TYPE_SPEED && can_signal_status_[k].timeout_flag_) {
        can_diag_state_[CANSignal_Diagnosis_Speed_Timeout].cur_ =
            CANSignal_Diagnosis_Status_Fail;
      } else if (k == CAN_FRAME_TYPE_YAW_RATE &&
                 can_signal_status_[k].timeout_flag_) {
        can_diag_state_[CANSignal_Diagnosis_Yawrate_Timeout].cur_ =
            CANSignal_Diagnosis_Status_Fail;
      }
    }
  }
#endif

  can_frame =
      CANFrameSummary(time_stamp, can_frame_raw.data(), can_frame_raw.size());
  {
#if CODE_LEVEL > LITE_CODE_LEVEL
    int verbose = LOG_VERBOSE_LOWEST;
    if (VLOG_IS_ON(verbose)) {
      const char *gear_str =
          GearTypeToString(can_frame.data_[CANRawDataType_Gear].gt.type);
      const char *ts_str = TurnSignalTypeToString(
          can_frame.data_[CANRawDataType_TurnSignal].ts.type);
      const char *power_mode_str = PowerModeTypeToString(
          can_frame.data_[CANRawDataType_power_mode].power_mode.power_mode_);
      const char *hazard_light_str =
          HazardLightTypeToString(can_frame.data_[CANRawDataType_hazard_light]
                                      .hazard_light.hazard_light_);
      const char *head_wiper_str = HeadWiperTypeToString(
          can_frame.data_[CANRawDataType_head_wiper].head_wiper.head_wiper_);
      const char *light_level_str = LightLevelToString(
          can_frame.data_[CANRawDataType_light_level].light_level.light_level_);
      const char *hmw_enable_str = WarnEnableTypeToString(
          can_frame.data_[CANRawDataType_hmw_enable].hmw_enable.hmw_enable_);
      const char *fcw_enable_str = WarnEnableTypeToString(
          can_frame.data_[CANRawDataType_fcw_enable].fcw_enable.fcw_enable_);
      const char *ldw_enable_str = WarnEnableTypeToString(
          can_frame.data_[CANRawDataType_ldw_enable].ldw_enable.ldw_enable_);
      const char *pcw_enable_str = WarnEnableTypeToString(
          can_frame.data_[CANRawDataType_pcw_enable].pcw_enable.pcw_enable_);
      const char *ufcw_enable_str = WarnEnableTypeToString(
          can_frame.data_[CANRawDataType_ufcw_enable].ufcw_enable.ufcw_enable_);

#ifdef _WIN32
      int print_every = 20;
#else
      int print_every = 40;
#endif

#if (GOOGLE_STRIP_LOG == 0)
      VLOG_EVERY_N(verbose, print_every)
          << "speed: " << can_frame.data_[CANRawDataType_Speed].sp.speed
          << ", gear type: " << gear_str << ", wheel angle: "
          << can_frame.data_[CANRawDataType_WheelAngle].wa.angle
          << ", turn signal: " << ts_str
          << ", acc: " << can_frame.data_[CANRawDataType_WheelAngle].acc.acc_
          << ", yaw rate: "
          << can_frame.data_[CANRawDataType_YawRate].yaw_rate.yaw_rate_
          << ", yaw rate offset: "
          << can_frame.data_[CANRawDataType_YawRateOffset]
                 .yaw_rate_offset.yaw_rate_offset_
          << ", lateral acc: "
          << can_frame.data_[CANRawDataType_lateral_acc]
                 .lateral_acc.lateral_acc_
          << ", brake pedal position: "
          << can_frame.data_[CANRawDataType_brake_pedal_position]
                 .brake_pedal_position.brake_pedal_position_
          << ", acc pedal position: "
          << can_frame.data_[CANRawDataType_acc_pedal_position]
                 .acc_pedal_position.acc_pedal_position_
          << ", power mode: " << power_mode_str
          << ", hazard light: " << hazard_light_str
          << ", head wiper: " << head_wiper_str << ", outside temperatrue: "
          << can_frame.data_[CANRawDataType_outside_temperatrue]
                 .outside_temperatrue.outside_temperatrue_
          << ", light level: " << light_level_str
          << ", hmw enable: " << hmw_enable_str
          << ", fcw enable: " << fcw_enable_str
          << ", ldw enable: " << ldw_enable_str
          << ", pcw enable: " << pcw_enable_str
          << ", ufcw enable: " << ufcw_enable_str
          << ", wheel speed front left: "
          << can_frame.data_[CANRawDataType_ufcw_enable]
                 .ws_front_left.ws_front_left_
          << ", wheel speed front right: "
          << can_frame.data_[CANRawDataType_ufcw_enable]
                 .ws_front_right.ws_front_right_
          << ", wheel speed rear left: "
          << can_frame.data_[CANRawDataType_ufcw_enable]
                 .ws_rear_left.ws_rear_left_
          << ", wheel speed rear right: "
          << can_frame.data_[CANRawDataType_ws_rear_right]
                 .ws_rear_right.ws_rear_right_
          << ", Yaw Rate Direction:"
          << can_frame.data_[CANRawDataType_YawRateDirection]
                 .yaw_rate_direction.type
          << ",Wheel Angle Direction:"
          << can_frame.data_[CANRawDataType_WheelAngleDirection]
                 .wa_direction.type
          << ",ws_front_left_valid :"
          << can_frame.data_[CANRawDataType_ws_front_left_valid]
                 .ws_front_left_valid.ws_valid
          << ",ws_front_right_valid :"
          << can_frame.data_[CANRawDataType_ws_front_right_valid]
                 .ws_front_right_valid.ws_valid
          << ",ws_rear_left_valid :"
          << can_frame.data_[CANRawDataType_ws_rear_left_valid]
                 .ws_rear_left_valid.ws_valid
          << ",ws_rear_right_valid :"
          << can_frame.data_[CANRawDataType_ws_rear_right_valid]
                 .ws_rear_right_valid.ws_valid
          << ",ws_front_left_dir :"
          << can_frame.data_[CANRawDataType_ws_front_left_dir]
                 .ws_front_left_dir.ws_dir
          << ",ws_front_right_dir :"
          << can_frame.data_[CANRawDataType_ws_front_right_dir]
                 .ws_front_right_dir.ws_dir
          << ",ws_rear_left_dir :"
          << can_frame.data_[CANRawDataType_ws_rear_left_dir]
                 .ws_rear_left_dir.ws_dir
          << ",ws_rear_right_dir :"
          << can_frame.data_[CANRawDataType_ws_rear_right_dir]
                 .ws_rear_right_dir.ws_dir
          << ",WheelAngleValid :"
          << can_frame.data_[CANRawDataType_WheelAngleValid]
                 .WheelAngleValid.wa_valid
          << ",YawRateValid :"
          << can_frame.data_[CANRawDataType_YawRateValid]
                 .YawRateValid.yawrate_valid
          << ",WheelPulseFL :"
          << can_frame.data_[CANRawDataType_WheelPulseFL].WheelPulseFL.whl_pule
          << ",WheelPulseFR :"
          << can_frame.data_[CANRawDataType_WheelPulseFR].WheelPulseFR.whl_pule
          << ",WheelPulseRL :"
          << can_frame.data_[CANRawDataType_WheelPulseRL].WheelPulseRL.whl_pule
          << ",WheelPulseRR :"
          << can_frame.data_[CANRawDataType_WheelPulseRR].WheelPulseRR.whl_pule
          << ",WheelPulseFLDir :"
          << can_frame.data_[CANRawDataType_WheelPulseFLDir]
                 .WheelPulseFLDir.whl_pule_dir
          << ",WheelPulseFRDir :"
          << can_frame.data_[CANRawDataType_WheelPulseFRDir]
                 .WheelPulseFRDir.whl_pule_dir
          << ",WheelPulseRLDir :"
          << can_frame.data_[CANRawDataType_WheelPulseRLDir]
                 .WheelPulseRLDir.whl_pule_dir
          << ",WheelPulseRRDir :"
          << can_frame.data_[CANRawDataType_WheelPulseRRDir]
                 .WheelPulseRRDir.whl_pule_dir
          << ",WheelPulseFLValid :"
          << can_frame.data_[CANRawDataType_WheelPulseFLValid]
                 .WheelPulseFLValid.whl_pule_valid
          << ",WheelPulseFRValid :"
          << can_frame.data_[CANRawDataType_WheelPulseFRValid]
                 .WheelPulseFRValid.whl_pule_valid
          << ",WheelPulseRLValid :"
          << can_frame.data_[CANRawDataType_WheelPulseRLValid]
                 .WheelPulseRLValid.whl_pule_valid
          << ",WheelPulseRRValid :"
          << can_frame.data_[CANRawDataType_WheelPulseRRValid]
                 .WheelPulseRRValid.whl_pule_valid;
#endif
    }
#endif
  }
  // set motion frame from can bus
  /*   motion_frame.speed = can_frame.data_[CANRawDataType_Speed].sp.speed;
     motion_frame.acc = can_frame.data_[CANRawDataType_Acc].acc.acc_;
     motion_frame.yaw_rate =
     can_frame.data_[CANRawDataType_YawRate].yaw_rate.yaw_rate_;
     motion_frame.time_stamp =
     can_frame.data_[CANRawDataType_YawRate].yaw_rate.time_stamp_;*/
  return true;
}

CANFrame CANParser::CANFrameSummary(const TimeStamp &target_ts,
                                    const CANFrameRaw *can_frame_raw,
                                    size_t can_count) {
  CANFrame canframe;
  int64_t can_dt[CANRawDataType_COUNT];
  int8_t can_rst_flg[CANRawDataType_COUNT];
  for (int i = 0; i < CANRawDataType_COUNT; i++) {
    can_dt[i] = INT_MAX;
    can_rst_flg[i] = 1;
  }

#if 0
  float speed_val = FLT_EPSILON;
  int speed_count = 0;
  const TurnSignal *can_ts = NULL;
  const Gear *can_gt = NULL;
  const VehSpeed *can_sp = NULL;
  const WheelAngle *can_wa = NULL;
  const CANAcc *can_acc = NULL;
  const CANYawRate *can_yaw_rate = NULL;



  for (size_t i = 0; i < can_count; i++) {
    const CANFrameRaw &raw = can_frame_raw[i];

    for (int j = 0; j < raw.info_count; j++) {
      if (raw.type[j] == CAN_FRAME_TYPE_SPEED) {
        speed_val += raw.data[j].sp.speed;
        speed_count++;
#ifdef __QNX__
        int64_t dt = std::llabs(raw.data[j].sp.time_stamp_ - target_ts);
#else
        int64_t dt = llabs(raw.data[j].sp.time_stamp_ - target_ts);
#endif
        if (dt < can_dt[CAN_FRAME_TYPE_SPEED]) {
          can_dt[CAN_FRAME_TYPE_SPEED] = dt;
          can_sp = &raw.data[j].sp;
        }

        // VLOG(0) << "speed: " << raw.data[j].sp.speed
        //        << " speed ts: " << raw.data[j].sp.time_stamp_
        //        << " dt: " << raw.data[j].sp.time_stamp_ - target_ts;
      } else if (raw.type[j] == CAN_FRAME_TYPE_GEAR) {
#ifdef __QNX__
        int64_t dt = std::llabs(raw.data[j].gt.time_stamp_ - target_ts);
#else
        int64_t dt = llabs(raw.data[j].gt.time_stamp_ - target_ts);
#endif
        if (dt < can_dt[CAN_FRAME_TYPE_GEAR]) {
          can_dt[CAN_FRAME_TYPE_GEAR] = dt;
          can_gt = &raw.data[j].gt;
        }
      } else if (raw.type[j] == CAN_FRAME_TYPE_WHEEL_ANGLE) {
#ifdef __QNX__
        int64_t dt = std::llabs(raw.data[j].wa.time_stamp_ - target_ts);
#else
        int64_t dt = llabs(raw.data[j].wa.time_stamp_ - target_ts);
#endif
        if (dt < can_dt[CAN_FRAME_TYPE_WHEEL_ANGLE]) {
          can_dt[CAN_FRAME_TYPE_WHEEL_ANGLE] = dt;
          can_wa = &raw.data[j].wa;
        }
      } else if (raw.type[j] == CAN_FRAME_TYPE_TURN_SIGNAL) {
#ifdef __QNX__
        int64_t dt = std::llabs(raw.data[j].ts.time_stamp_ - target_ts);
#else
        int64_t dt = llabs(raw.data[j].ts.time_stamp_ - target_ts);
#endif
        if (dt < can_dt[CAN_FRAME_TYPE_TURN_SIGNAL]) {
          can_dt[CAN_FRAME_TYPE_TURN_SIGNAL] = dt;
          can_ts = &raw.data[j].ts;
        }

        const TurnSignal *turn_signal = &raw.data[j].ts;

        if (first_ts_status_) {
          last_ts_status_ = *turn_signal;
          first_ts_status_ = false;
        } else {
          if (last_ts_status_.type == turn_signal->type) {
            last_ts_status_ = *turn_signal;
          } else {
            if (turn_signal->type != TURN_SIGNAL_NONE) {
              last_ts_status_ = *turn_signal;
            } else if (turn_signal->time_stamp_ - last_ts_status_.time_stamp_
                       > turn_signal_interval_ * 2) {
              last_ts_status_ = *turn_signal;
            } else {
              // do nothing
            }
          }
        }
      } else if (raw.type[j] == CAN_FRAME_TYPE_ACC) {
#ifdef __QNX__
        int64_t dt = std::llabs(raw.data[j].acc.time_stamp_ - target_ts);
#else
        int64_t dt = llabs(raw.data[j].acc.time_stamp_ - target_ts);
#endif
        if (dt < can_dt[CAN_FRAME_TYPE_ACC]) {
          can_dt[CAN_FRAME_TYPE_ACC] = dt;
          can_acc = &raw.data[j].acc;
        }
      } else if (raw.type[j] == CAN_FRAME_TYPE_YAW_RATE) {
#ifdef __QNX__
        int64_t dt = std::llabs(raw.data[j].yaw_rate.time_stamp_ - target_ts);
#else
        int64_t dt = llabs(raw.data[j].yaw_rate.time_stamp_ - target_ts);
#endif
        if (dt < can_dt[CAN_FRAME_TYPE_YAW_RATE]) {
          can_dt[CAN_FRAME_TYPE_YAW_RATE] = dt;
          can_yaw_rate = &raw.data[j].yaw_rate;
        }
      }
    }  // end of for j
  }  // end of for i

  if (speed_count > 0) {
    speed_val /= speed_count;
  }

  // TODO(dev): find a way to smooth can frames
  {
    if (can_sp) {
      canframe.data_[CANRawDataType_Speed].sp.speed = speed_val;
      canframe.data_[CANRawDataType_Speed].sp.time_stamp_ = can_sp->time_stamp_;
    }

    if (can_gt) {
      canframe.data_[CANRawDataType_Gear].gt.type = can_gt->type;
      canframe.data_[CANRawDataType_Gear].gt.time_stamp_ = can_gt->time_stamp_;
    }

    if (can_wa) {
      canframe.data_[CANRawDataType_WheelAngle].wa.angle = can_wa->angle;
      canframe.data_[CANRawDataType_WheelAngle].wa.time_stamp_ =
        can_wa->time_stamp_;
    }

    if (can_acc) {
      canframe.data_[CANRawDataType_Acc].acc = *can_acc;
    }

    if (can_yaw_rate) {
      canframe.data_[CANRawDataType_YawRate].yaw_rate = *can_yaw_rate;
    }

    // turn signal
    {
      canframe.data_[CANRawDataType_TurnSignal].ts.type = last_ts_status_.type;
      canframe.data_[CANRawDataType_TurnSignal].ts.time_stamp_ = can_ts
        ? can_ts->time_stamp_
        : last_ts_status_.time_stamp_;
    }
  }
#else
  for (size_t i = 0; i < can_count; i++) {
    const CANFrameRaw &raw = can_frame_raw[i];
    for (int j = 0; j < raw.info_count; j++) {
      if ((raw.type[j] >= 0) && (raw.type[j] < CANRawDataType_COUNT)) {
        int64_t dt;
        int64_t ts;
        // RawInfo is union type, and time_stamp_ is at first 8 bytes.
        ts = raw.data[j].general_int.time_stamp_;
#ifdef __QNX__
        dt = std::llabs(ts - target_ts);
#else
        dt = llabs(ts - target_ts);
#endif
        if (dt < can_dt[raw.type[j]]) {
          can_dt[raw.type[j]] = dt;
          CANFrameFilterFunc filter = can_filter_func_[raw.type[j]];
          if (filter != NULL) {
            canframe.data_[raw.type[j]] =
                (this->*filter)(can_rst_flg[raw.type[j]], raw.data[j]);
          } else {
            canframe.data_[raw.type[j]] = raw.data[j];
          }
        }  // end of if
      }    // end of if
    }      // end of for j
  }        // end of for i

#endif
  return canframe;
}

int CANParser::GetCanFilter(struct can_filter *filter, int size) {
#if 1
  int i = 0;
  int filtersize = 0;

  for (auto it = frame_map_.begin(); it != frame_map_.end(); ++it) {
    filter[i].can_id = it->first;
    filter[i].can_mask = CAN_SFF_MASK;
    if (i < CANRawDataType_COUNT) {
      i++;
    }  // end of if
  }    // end of for

  if (i > 0) {
    // remove repeated can id
    int j = 0, k = 0, m = 0;
    for (j = m = 1; j < i; j++) {
      for (k = 0; k < m; k++) {
        if (filter[k].can_id == filter[j].can_id) {
          break;
        }
      }
      if (k == m) {
        filter[m].can_id = filter[j].can_id;
        filter[m].can_mask = filter[j].can_mask;
        m++;
      }
    }  // end of for
    filtersize = m;
  }  // end of if

  return filtersize;
#else

  int i = 0;
  for (auto it = frame_map_.begin(); it != frame_map_.end(); ++it) {
    filter[i].can_id = it->first;
    filter[i].can_mask = CAN_SFF_MASK;
    i++;
  }
  return frame_map_.size();
#endif
}

static inline int32 ReverseEndian(const int32 &data) {
  int32 rdata = ((data & 0xFF000000) >> 24) | ((data & 0x00FF0000) >> 8) |
                ((data & 0x0000FF00) << 8) | ((data & 0x000000FF) << 24);
  return rdata;
}

static inline uint32 ReverseEndian(const uint32 &data) {
  uint32 rdata = ((data & 0xFF000000) >> 24) | ((data & 0x00FF0000) >> 8) |
                 ((data & 0x0000FF00) << 8) | ((data & 0x000000FF) << 24);
  return rdata;
}

static inline uint64 ReverseEndian(const uint64 &data) {
  uint64 rdata =
      (data & 0xFF00000000000000LL) >> 56 |
      (data & 0x00FF000000000000LL) >> 40 |
      (data & 0x0000FF0000000000LL) >> 24 | (data & 0x000000FF00000000LL) >> 8 |
      (data & 0x00000000FF000000LL) << 8 | (data & 0x0000000000FF0000LL) << 24 |
      (data & 0x000000000000FF00LL) << 40 | (data & 0x00000000000000FFLL) << 56;
  return rdata;
}

bool CANParser::ParseRawData(const struct can_frame *frame,
                             const CANSettingMap *setting_map,
                             CANRawDataMap &raw_data_map) {
  CANSettingMap::const_iterator setting_iter = frame_map_.find(frame->can_id);
  if (setting_iter == frame_map_.end()) {
    return false;
  }

  const std::vector<CANSetting> &setting_vec = setting_iter->second;
  if (setting_vec.empty()) {
    return false;
  }

  int endian = setting_vec[0].endian_;

  uint64_t data = 0;
  // resolve endian
  // It's better don't to use reinterpret_cast to get unit64 data
  // from frame->data array memory since the cpu endian could be
  // Intel or Motorola in different HW.
  if (endian == CANRawEndianType_Big) {
    for (int i = 0; i < 8; i++) {  // Motorola Format
      // msb is data[0], lsb  is data[7]
      data = (data << 8) | (frame->data[i] & 0xFF);
    }
  } else {  // default is Intel format
    for (int i = 0; i < 8; i++) {
      // msb is data[7], lsb  is data[0]
      data = (data << 8) | (frame->data[7 - i] & 0xFF);
    }
  }
  // VLOG(LOG_VERBOSE_HIGH) << "CANParser::ParseRawData: data = "
  //  << std::hex << data;

  for (size_t i = 0; i < setting_vec.size(); i++) {
    const CANSetting *setting = &setting_vec[i];
    int bit_len = setting->bit_length_;
    uint64_t bit_mask = data_mask_[bit_len];
    uint64_t unsigned_data = (data >> setting->start_bit_) & bit_mask;

    CANRawData raw;
    if (setting->value_Map_.size() > 0) {
      raw.value_map_ = &setting->value_Map_;
    }
    switch (setting->value_type_) {
      case CANRawDataValue_Signed: {
        unsigned_data <<= (64 - bit_len);
        int64_t signed_data = static_cast<int64_t>(unsigned_data);
        signed_data >>= (64 - bit_len);
        if (raw.value_map_) {
          raw.value_type_ = CANRawDataValue_Signed;
          raw.value_.u32 = static_cast<int32_t>(signed_data * setting->scale_ +
                                                setting->offset_);
        } else {
          raw.value_type_ = CANRawDataValue_Float;
          raw.value_.f32 = signed_data * setting->scale_ + setting->offset_;
        }
      } break;
      case CANRawDataValue_Float: {
        uint32_t data32 = static_cast<uint32_t>(unsigned_data & 0xFFFFFFFF);
        float float_data = *(reinterpret_cast<float *>(&data32));
        raw.value_type_ = CANRawDataValue_Float;
        raw.value_.f32 = float_data * setting->scale_ + setting->offset_;
      } break;
      case CANRawDataValue_Double: {
        double double_data = *(reinterpret_cast<double *>(&unsigned_data));
        raw.value_type_ = CANRawDataValue_Double;
        raw.value_.f64 = double_data * setting->scale_ + setting->offset_;
      } break;
      case CANRawDataValue_Enum:
      case CANRawDataValue_Unsigned:
      default: {
        if (raw.value_map_) {
          raw.value_type_ = CANRawDataValue_Unsigned;
          raw.value_.i32 = static_cast<int32_t>(
              unsigned_data * setting->scale_ + setting->offset_);
        } else {
          raw.value_type_ = CANRawDataValue_Float;
          raw.value_.f32 = unsigned_data * setting->scale_ + setting->offset_;
        }
      } break;
    }
    switch (setting->raw_type_) {
      case CANRawDataType_Speed: {
        if ((raw.value_.f32 >= setting->min_) &&
            (raw.value_.f32 <= setting->max_)) {
          can_diag_state_[CANSignal_Diagnosis_Speed].cur_ =
              CANSignal_Diagnosis_Status_Pass;
        } else {
          can_diag_state_[CANSignal_Diagnosis_Speed].cur_ =
              CANSignal_Diagnosis_Status_Fail;
        }
      } break;
      case CANRawDataType_YawRate: {
        if ((raw.value_.f32 >= setting->min_) &&
            (raw.value_.f32 <= setting->max_)) {
          can_diag_state_[CANSignal_Diagnosis_Yawrate].cur_ =
              CANSignal_Diagnosis_Status_Pass;
        } else {
          can_diag_state_[CANSignal_Diagnosis_Yawrate].cur_ =
              CANSignal_Diagnosis_Status_Fail;
        }
      } break;
      default:
        break;
    }

    raw_data_map[setting->raw_type_] = raw;
  }

  return true;
}

int CANParser::ParseWheelAngle(const CANRawDataMap &raw_data_map,
                               CANFrameRaw &raw) {
  float angle = 0;

  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_WheelAngle);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  angle = iter->second.value_.f32;

  iter = raw_data_map.find(CANRawDataType_WheelAngleDirection);
  if ((iter != raw_data_map.end()) && iter->second.value_map_ != NULL) {
    CANValueMap::const_iterator iter_map;
    if (iter->second.value_type_ == CANRawDataValue_Signed) {
      iter_map = iter->second.value_map_->find(iter->second.value_.i32);
      if (iter_map != iter->second.value_map_->end()) {
        if (iter_map->second == DirectionType_Negative) {  // Negative value
          angle = -fabsf(angle);
        } else {  // positive value
          angle = fabsf(angle);
        }
      }
    } else {
      iter_map = iter->second.value_map_->find(iter->second.value_.u32);
      if (iter_map != iter->second.value_map_->end()) {
        if (iter_map->second == DirectionType_Negative) {  // Negative value
          angle = -fabsf(angle);
        } else {  // positive value
          angle = fabsf(angle);
        }
      }
    }
  }

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_WHEEL_ANGLE;
  raw.data[idx].wa.angle = angle;
  raw.data[idx].wa.time_stamp_ = raw.raw_ts;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "wheel angle " << angle;
  return 1;
}

int CANParser::ParseWheelAngleDirection(const CANRawDataMap &raw_data_map,
                                        CANFrameRaw &raw) {
  return 1;
}

int CANParser::ParseGear(const CANRawDataMap &raw_data_map, CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_Gear);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "gear signal ";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      // LOG(ERROR) << str_buf <<"value not found, "
      //  << iter_raw->second.value_.i32;
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      // LOG(ERROR) << str_buf << "value not found, "
      // << iter_raw->second.value_.u32;
      return 0;
    }
  }

  int type = iter_map->second;

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_GEAR;
  raw.data[idx].gt.time_stamp_ = raw.raw_ts;
  raw.data[idx].gt.type = type;
  raw.info_count++;

  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << type;
  return 1;
}

int CANParser::ParseVehSpeed(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_Speed);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  float speed = 0;
  speed = iter->second.value_.f32;

  iter = raw_data_map.find(CANRawDataType_VehicleSpeedValid);
  bool speed_valid_flag = CANValidType_VALID;
  if ((iter != raw_data_map.end()) && iter->second.value_map_ != NULL) {
    CANValueMap::const_iterator iter_map;
    iter_map = iter->second.value_map_->find(iter->second.value_.u32);
    if (iter_map != iter->second.value_map_->end()) {
      if (iter_map->second == CANValidType_VALID) {
        speed_valid_flag = CANValidType_VALID;
      } else {
        speed_valid_flag = CANValidType_INVALID;
      }
    }
  }
  if (speed_valid_flag == CANValidType_VALID) {
    int idx = raw.info_count;
    raw.type[idx] = CAN_FRAME_TYPE_SPEED;
    raw.data[idx].sp.speed = speed;
    raw.data[idx].sp.time_stamp_ = raw.raw_ts;
    raw.info_count++;
    // VLOG(LOG_VERBOSE_HIGHEST) << "speed " << speed;
  }
  return 1;
}

int CANParser::ParseTurnSignal(const CANRawDataMap &raw_data_map,
                               CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_TurnSignal);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "turn signal ";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      // LOG(ERROR) << str_buf << "value not found, "
      //<< iter_raw->second.value_.i32;
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      // LOG(ERROR) << str_buf << "value not found, "
      //<< iter_raw->second.value_.u32;
      return 0;
    }
  }

  int idx = raw.info_count;
  int signal = iter_map->second;
  raw.type[idx] = CAN_FRAME_TYPE_TURN_SIGNAL;
  raw.data[idx].ts.time_stamp_ = raw.raw_ts;
  raw.data[idx].ts.type = signal;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << signal;
  return 1;
}

int CANParser::ParseUDLCLeftStatus(const CANRawDataMap &raw_data_map,
                                   CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_UDLCLeftStatus);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "UDLCLeftStatus signal ";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      // LOG(ERROR) << str_buf << "value not found, "
      //<< iter_raw->second.value_.i32;
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      // LOG(ERROR) << str_buf << "value not found, "
      //<< iter_raw->second.value_.u32;
      return 0;
    }
  }

  int idx = raw.info_count;
  int signal = iter_map->second;
  raw.type[idx] = CAN_FRAME_TYPE_UDLCLeftStatus;
  raw.data[idx].ts.time_stamp_ = raw.raw_ts;
  raw.data[idx].ts.type = signal;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << signal;
  return 1;
}

int CANParser::ParseUDLCRightStatus(const CANRawDataMap &raw_data_map,
                                    CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_UDLCRightStatus);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "UDLCRightStatus signal ";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value not found, "
        << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      // LOG(ERROR) << str_buf << "value not found, "
      //  << iter_raw->second.value_.u32;
      return 0;
    }
  }

  int idx = raw.info_count;
  int signal = iter_map->second;
  raw.type[idx] = CAN_FRAME_TYPE_UDLCRightStatus;
  raw.data[idx].ts.time_stamp_ = raw.raw_ts;
  raw.data[idx].ts.type = signal;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << signal;
  return 1;
}

int CANParser::ParseLeftTurnSignal(const CANRawDataMap &raw_data_map,
                                   CANFrameRaw &raw) {
  int has_lt = 1;
  int has_rt = 1;
  CANRawDataMap::const_iterator iter_raw;
  CANValueMap::const_iterator iter_map;
  int left_turn = 0;
  int right_turn = 0;
  int signal = 0;
  int idx = raw.info_count;

  std::string str_buf = "left turn signal ";
  iter_raw = raw_data_map.find(CANRawDataType_LeftTurnSignal);
  if (iter_raw != raw_data_map.end() && iter_raw->second.value_map_) {
    if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
      iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
      if (iter_map == iter_raw->second.value_map_->end()) {
        /*LOG(ERROR) << str_buf << "value not found, "
          << iter_raw->second.value_.i32;*/
        has_lt = 0;
      }
    } else {
      iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
      if (iter_map == iter_raw->second.value_map_->end()) {
        /*LOG(ERROR) << str_buf << "value not found, "
          << iter_raw->second.value_.u32;*/
        has_lt = 0;
      }
    }
  } else {
    has_lt = 0;
  }
  if (has_lt == 1) {
    left_turn = iter_map->second;
  }

  iter_raw = raw_data_map.find(CANRawDataType_RightTurnSignal);
  std::string str_buf1 = "right turn signal ";
  if (iter_raw != raw_data_map.end() && iter_raw->second.value_map_) {
    if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
      iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
      if (iter_map == iter_raw->second.value_map_->end()) {
        /*LOG(ERROR) << str_buf1 << "value not found, "
          << iter_raw->second.value_.i32;*/
        has_rt = 0;
      }
    } else {
      iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
      if (iter_map == iter_raw->second.value_map_->end()) {
        /* LOG(ERROR) << str_buf1 << "value not found, "
           << iter_raw->second.value_.u32;*/
        has_rt = 0;
      }
    }
  } else {
    has_rt = 0;
  }

  if (has_rt == 1) {
    right_turn = iter_map->second;
  }

  if (has_lt) {
    signal = (left_turn | (right_turn << 1));
    raw.type[idx] = CAN_FRAME_TYPE_TURN_SIGNAL;
    raw.data[idx].ts.time_stamp_ = raw.raw_ts;
    raw.data[idx].ts.type = signal;
    raw.info_count++;
    // VLOG(LOG_VERBOSE_HIGHEST) << str_buf1 << right_turn;
  }

  return 1;
}

int CANParser::ParseRightTurnSignal(const CANRawDataMap &raw_data_map,
                                    CANFrameRaw &raw) {
  int has_lt = 1;
  int has_rt = 1;
  CANRawDataMap::const_iterator iter_raw;
  CANValueMap::const_iterator iter_map;
  int left_turn = 0;
  int right_turn = 0;
  int signal = 0;
  int idx = raw.info_count;
  std::string str_buf = "left turn signal ";
  iter_raw = raw_data_map.find(CANRawDataType_LeftTurnSignal);
  if (iter_raw != raw_data_map.end() && iter_raw->second.value_map_) {
    if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
      iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
      if (iter_map == iter_raw->second.value_map_->end()) {
        /*LOG(ERROR) << str_buf << "value not found, "
          << iter_raw->second.value_.i32;*/
        has_lt = 0;
      }
    } else {
      iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
      if (iter_map == iter_raw->second.value_map_->end()) {
        /*LOG(ERROR) << str_buf << "value not found, "
          << iter_raw->second.value_.u32;*/
        has_lt = 0;
      }
    }
  } else {
    has_lt = 0;
  }

  if (has_lt == 1) {
    left_turn = iter_map->second;
  }

  iter_raw = raw_data_map.find(CANRawDataType_RightTurnSignal);
  std::string str_buf1 = "right turn signal ";
  if (iter_raw != raw_data_map.end() && iter_raw->second.value_map_) {
    if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
      iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
      if (iter_map == iter_raw->second.value_map_->end()) {
        /*LOG(ERROR) << str_buf1 << "value not found, "
          << iter_raw->second.value_.i32;*/
        has_rt = 0;
      }
    } else {
      iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
      if (iter_map == iter_raw->second.value_map_->end()) {
        /*LOG(ERROR) << str_buf1 << "value not found, "
          << iter_raw->second.value_.u32;*/
        has_rt = 0;
      }
    }
  } else {
    has_rt = 0;
  }

  if (has_rt == 1) {
    right_turn = iter_map->second;
  }

  if (has_lt == 0 && has_rt == 1) {
    signal = (left_turn | (right_turn << 1));
    raw.type[idx] = CAN_FRAME_TYPE_TURN_SIGNAL;
    raw.data[idx].ts.time_stamp_ = raw.raw_ts;
    raw.data[idx].ts.type = signal;
    raw.info_count++;
    // VLOG(LOG_VERBOSE_HIGHEST) << str_buf1 << right_turn;
  }
  return 1;
}

int CANParser::ParseAcc(const CANRawDataMap &raw_data_map, CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_Acc);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  float acc = 0;
  acc = iter->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_ACC;
  raw.data[idx].acc.time_stamp_ = raw.raw_ts;
  raw.data[idx].acc.acc_ = acc;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "acc " << acc;
  return 1;
}

int CANParser::ParseYawRateValid(const CANRawDataMap &raw_data_map,
                                 CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_YawRateValid);
  if (iter == raw_data_map.end()) {
    return 0;
  }
  bool yawrate_valid_flag = CANValidType_VALID;
  if ((iter != raw_data_map.end()) && iter->second.value_map_ != NULL) {
    CANValueMap::const_iterator iter_map;
    iter_map = iter->second.value_map_->find(iter->second.value_.u32);
    if (iter_map != iter->second.value_map_->end()) {
      if (iter_map->second == CANValidType_VALID) {
        yawrate_valid_flag = CANValidType_VALID;
      } else {
        yawrate_valid_flag = CANValidType_INVALID;
      }
    }
  }
  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_YawRateValid;
  raw.data[idx].YawRateValid.time_stamp_ = raw.raw_ts;
  raw.data[idx].YawRateValid.yawrate_valid = yawrate_valid_flag;
  raw.info_count++;
}

int CANParser::ParseYawRate(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_YawRate);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  float yaw_rate = 0;
  yaw_rate = iter->second.value_.f32;

  iter = raw_data_map.find(CANRawDataType_YawRateOffset);
  if (iter != raw_data_map.end()) {
    float yaw_rate_offset = 0;
    yaw_rate_offset = iter->second.value_.f32;
    yaw_rate += yaw_rate_offset;
  }

  iter = raw_data_map.find(CANRawDataType_YawRateDirection);

  if ((iter != raw_data_map.end()) && iter->second.value_map_ != NULL) {
    CANValueMap::const_iterator iter_map;
    if (iter->second.value_type_ == CANRawDataValue_Signed) {
      iter_map = iter->second.value_map_->find(iter->second.value_.i32);
      if (iter_map != iter->second.value_map_->end()) {
        if (iter_map->second == DirectionType_Negative) {  // Negative value
          yaw_rate = -fabsf(yaw_rate);
        } else {  // positive value
          yaw_rate = fabsf(yaw_rate);
        }
      }
    } else {
      iter_map = iter->second.value_map_->find(iter->second.value_.u32);
      if (iter_map != iter->second.value_map_->end()) {
        if (iter_map->second == DirectionType_Negative) {  // Negative value
          yaw_rate = -fabsf(yaw_rate);
        } else {  // positive value
          yaw_rate = fabsf(yaw_rate);
        }
      }
    }
  }

  iter = raw_data_map.find(CANRawDataType_YawRateValid);
  bool yawrate_valid_flag = CANValidType_VALID;
  if ((iter != raw_data_map.end()) && iter->second.value_map_ != NULL) {
    CANValueMap::const_iterator iter_map;
    iter_map = iter->second.value_map_->find(iter->second.value_.u32);
    if (iter_map != iter->second.value_map_->end()) {
      if (iter_map->second == CANValidType_VALID) {
        yawrate_valid_flag = CANValidType_VALID;
      } else {
        yawrate_valid_flag = CANValidType_INVALID;
      }
    }
  }
  if (yawrate_valid_flag == CANValidType_VALID) {
    int idx = raw.info_count;
    raw.type[idx] = CAN_FRAME_TYPE_YAW_RATE;
    raw.data[idx].yaw_rate.time_stamp_ = raw.raw_ts;
    raw.data[idx].yaw_rate.yaw_rate_ = yaw_rate;
    raw.info_count++;
    // VLOG(LOG_VERBOSE_HIGHEST) << "yaw rate " << yaw_rate;
  }
  return 1;
}

int CANParser::ParseYawRateOffset(const CANRawDataMap &raw_data_map,
                                  CANFrameRaw &raw) {
  return 1;
}

int CANParser::ParseYawRateDirection(const CANRawDataMap &raw_data_map,
                                     CANFrameRaw &raw) {
  return 1;
}

int CANParser::ParseLateralAcc(const CANRawDataMap &raw_data_map,
                               CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_lateral_acc);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  float lateral_acc = 0;
  lateral_acc = iter->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_LATERAL_ACC;
  raw.data[idx].lateral_acc.time_stamp_ = raw.raw_ts;
  raw.data[idx].lateral_acc.lateral_acc_ = lateral_acc;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "lateral acc " << lateral_acc;
  return 1;
}

int CANParser::ParseBrakePedalPosition(const CANRawDataMap &raw_data_map,
                                       CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_brake_pedal_position);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  float brake_pedal_position = 0;
  brake_pedal_position = iter->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_BRAKE_PEDAL_POSITION;
  raw.data[idx].brake_pedal_position.time_stamp_ = raw.raw_ts;
  raw.data[idx].brake_pedal_position.brake_pedal_position_ =
      brake_pedal_position;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "brake pedal position" <<
  // brake_pedal_position;
  return 1;
}

int CANParser::ParseAccPedalPosition(const CANRawDataMap &raw_data_map,
                                     CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_acc_pedal_position);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  float acc_pedal_position = 0;
  acc_pedal_position = iter->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_ACC_PEDAL_POSITION;
  raw.data[idx].acc_pedal_position.time_stamp_ = raw.raw_ts;
  raw.data[idx].acc_pedal_position.acc_pedal_position_ = acc_pedal_position;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "acc pedal position" << acc_pedal_position;
  return 1;
}

int CANParser::ParsePowerMode(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_power_mode);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "power mode signal ";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value  not found, "
        << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value  not found, "
        << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int idx = raw.info_count;
  int signal = iter_map->second;
  raw.type[idx] = CAN_FRAME_TYPE_POWER_MODE;
  raw.data[idx].power_mode.time_stamp_ = raw.raw_ts;
  raw.data[idx].power_mode.power_mode_ = signal;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << signal;
  return 1;
}

int CANParser::ParseHazardLight(const CANRawDataMap &raw_data_map,
                                CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_hazard_light);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "hazard light signal";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value  not found, "
        << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value not found, "
        << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int idx = raw.info_count;
  int signal = iter_map->second;
  raw.type[idx] = CAN_FRAME_TYPE_HAZARD_LIGHT;
  raw.data[idx].hazard_light.time_stamp_ = raw.raw_ts;
  raw.data[idx].hazard_light.hazard_light_ = static_cast<bool>(signal);
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << signal;
  return 1;
}

int CANParser::ParseHeadWiper(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_head_wiper);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "head wiper signal ";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value  not found, "
        << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value  not found, "
        << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int idx = raw.info_count;
  int signal = iter_map->second;
  raw.type[idx] = CAN_FRAME_TYPE_HEAD_WIPER;
  raw.data[idx].head_wiper.time_stamp_ = raw.raw_ts;
  raw.data[idx].head_wiper.head_wiper_ = signal;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << signal;
  return 1;
}

int CANParser::ParseOutsideTemperatrue(const CANRawDataMap &raw_data_map,
                                       CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_outside_temperatrue);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  float outside_temperatrue = 0;
  outside_temperatrue = iter->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_OUTSIDE_TEMPERATRUE;
  raw.data[idx].outside_temperatrue.time_stamp_ = raw.raw_ts;
  raw.data[idx].outside_temperatrue.outside_temperatrue_ = outside_temperatrue;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "outside temperatrue" <<
  // outside_temperatrue;
  return 1;
}

int CANParser::ParseLightLevel(const CANRawDataMap &raw_data_map,
                               CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_light_level);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "light level signal ";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /* LOG(ERROR) << str_buf << "value  not found, "
         << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /* LOG(ERROR) << str_buf << "value  not found, "
         << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int idx = raw.info_count;
  int signal = iter_map->second;
  raw.type[idx] = CAN_FRAME_TYPE_LIGHT_LEVEL;
  raw.data[idx].light_level.time_stamp_ = raw.raw_ts;
  raw.data[idx].light_level.light_level_ = signal;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << signal;
  return 1;
}

int CANParser::ParseHMWEnable(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_hmw_enable);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "hmw enable signal ";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value  not found, "
        << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << " value  not found, "
        << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int idx = raw.info_count;
  int signal = iter_map->second;
  raw.type[idx] = CAN_FRAME_TYPE_HMW_ENABLE;
  raw.data[idx].hmw_enable.time_stamp_ = raw.raw_ts;
  raw.data[idx].hmw_enable.hmw_enable_ = signal;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << signal;
  return 1;
}

int CANParser::ParseFCWEnable(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_fcw_enable);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "fcw enable signal ";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value  not found, "
        << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << " value  not found, "
        << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int idx = raw.info_count;
  int signal = iter_map->second;
  raw.type[idx] = CAN_FRAME_TYPE_HMW_ENABLE;
  raw.data[idx].fcw_enable.time_stamp_ = raw.raw_ts;
  raw.data[idx].fcw_enable.fcw_enable_ = signal;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << signal;
  return 1;
}

int CANParser::ParseLDWEnable(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_ldw_enable);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "ldw enable signal ";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value  not found, "
        << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /* LOG(ERROR) << str_buf << " value  not found, "
         << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int idx = raw.info_count;
  int signal = iter_map->second;
  raw.type[idx] = CAN_FRAME_TYPE_LDW_ENABLE;
  raw.data[idx].ldw_enable.time_stamp_ = raw.raw_ts;
  raw.data[idx].ldw_enable.ldw_enable_ = signal;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << signal;
  return 1;
}

int CANParser::ParsePCWEnable(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_pcw_enable);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "pcw enable signal ";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value  not found, "
        << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << " value  not found, "
        << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int idx = raw.info_count;
  int signal = iter_map->second;
  raw.type[idx] = CAN_FRAME_TYPE_PCW_ENABLE;
  raw.data[idx].pcw_enable.time_stamp_ = raw.raw_ts;
  raw.data[idx].pcw_enable.pcw_enable_ = signal;
  raw.info_count++;
  /*VLOG(LOG_VERBOSE_HIGHEST) << str_buf << signal;*/
  return 1;
}

int CANParser::ParseUFCWEnable(const CANRawDataMap &raw_data_map,
                               CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_ufcw_enable);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "ufcw enable signal ";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value  not found, "
        << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /* LOG(ERROR) << str_buf << " value  not found, "
         << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int idx = raw.info_count;
  int signal = iter_map->second;
  raw.type[idx] = CAN_FRAME_TYPE_UFCW_ENABLE;
  raw.data[idx].ufcw_enable.time_stamp_ = raw.raw_ts;
  raw.data[idx].ufcw_enable.ufcw_enable_ = signal;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << signal;
  return 1;
}

int CANParser::ParseWSFrontLeft(const CANRawDataMap &raw_data_map,
                                CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_ws_front_left);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  float ws_front_left = 0;
  ws_front_left = iter->second.value_.f32;

  // check direction
  iter = raw_data_map.find(CANRawDataType_ws_front_left_dir);
  if ((iter != raw_data_map.end()) && iter->second.value_map_ != NULL) {
    CANValueMap::const_iterator iter_map;
    if (iter->second.value_type_ == CANRawDataValue_Signed) {
      iter_map = iter->second.value_map_->find(iter->second.value_.i32);
      if (iter_map != iter->second.value_map_->end()) {
        if (iter_map->second == WHEEL_DIR_TYPE_INVALID) {  // Invalid value
          ws_front_left = 0.0f;
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_FORWARD) {  // forward value
          ws_front_left = fabsf(ws_front_left);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_BACKWARD) {  // backward value
          ws_front_left = -fabsf(ws_front_left);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_STANDSTILL) {  // standstill value
          ws_front_left = 0;
        } else {
        }
      }
    } else {
      iter_map = iter->second.value_map_->find(iter->second.value_.u32);
      if (iter_map != iter->second.value_map_->end()) {
        if (iter_map->second == WHEEL_DIR_TYPE_INVALID) {  // Invalid value
          ws_front_left = 0.0f;
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_FORWARD) {  // forward value
          ws_front_left = fabsf(ws_front_left);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_BACKWARD) {  // backward value
          ws_front_left = -fabsf(ws_front_left);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_STANDSTILL) {  // standstill value
          ws_front_left = 0;
        } else {
        }
      }
    }
  }

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_WS_FRONT_LEFT;
  raw.data[idx].ws_front_left.time_stamp_ = raw.raw_ts;
  raw.data[idx].ws_front_left.ws_front_left_ = ws_front_left;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "wheel speed front left" << ws_front_left;
  return 1;
}

int CANParser::ParseWSFrontRight(const CANRawDataMap &raw_data_map,
                                 CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_ws_front_right);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  float ws_front_right = 0;
  ws_front_right = iter->second.value_.f32;

  // check direction
  iter = raw_data_map.find(CANRawDataType_ws_front_right_dir);
  if ((iter != raw_data_map.end()) && iter->second.value_map_ != NULL) {
    CANValueMap::const_iterator iter_map;
    if (iter->second.value_type_ == CANRawDataValue_Signed) {
      iter_map = iter->second.value_map_->find(iter->second.value_.i32);
      if (iter_map != iter->second.value_map_->end()) {
        if (iter_map->second == WHEEL_DIR_TYPE_INVALID) {  // Invalid value
          ws_front_right = 0.0f;
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_FORWARD) {  // forward value
          ws_front_right = fabsf(ws_front_right);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_BACKWARD) {  // backward value
          ws_front_right = -fabsf(ws_front_right);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_STANDSTILL) {  // standstill value
          ws_front_right = 0;
        } else {
        }
      }
    } else {
      iter_map = iter->second.value_map_->find(iter->second.value_.u32);
      if (iter_map != iter->second.value_map_->end()) {
        if (iter_map->second == WHEEL_DIR_TYPE_INVALID) {  // Invalid value
          ws_front_right = 0.0f;
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_FORWARD) {  // forward value
          ws_front_right = fabsf(ws_front_right);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_BACKWARD) {  // backward value
          ws_front_right = -fabsf(ws_front_right);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_STANDSTILL) {  // standstill value
          ws_front_right = 0;
        } else {
        }
      }
    }
  }

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_WS_FRONT_RIGHT;
  raw.data[idx].ws_front_right.time_stamp_ = raw.raw_ts;
  raw.data[idx].ws_front_right.ws_front_right_ = ws_front_right;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "wheel speed front right" << ws_front_right;
  return 1;
}

int CANParser::ParseWSRearLeft(const CANRawDataMap &raw_data_map,
                               CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_ws_rear_left);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  float ws_rear_left = 0;
  ws_rear_left = iter->second.value_.f32;

  // check direction
  iter = raw_data_map.find(CANRawDataType_ws_rear_left_dir);
  if ((iter != raw_data_map.end()) && iter->second.value_map_ != NULL) {
    CANValueMap::const_iterator iter_map;
    if (iter->second.value_type_ == CANRawDataValue_Signed) {
      iter_map = iter->second.value_map_->find(iter->second.value_.i32);
      if (iter_map != iter->second.value_map_->end()) {
        if (iter_map->second == WHEEL_DIR_TYPE_INVALID) {  // Invalid value
          ws_rear_left = 0.0f;
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_FORWARD) {  // forward value
          ws_rear_left = fabsf(ws_rear_left);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_BACKWARD) {  // backward value
          ws_rear_left = -fabsf(ws_rear_left);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_STANDSTILL) {  // standstill value
          ws_rear_left = 0;
        } else {
        }
      }
    } else {
      iter_map = iter->second.value_map_->find(iter->second.value_.u32);
      if (iter_map != iter->second.value_map_->end()) {
        if (iter_map->second == WHEEL_DIR_TYPE_INVALID) {  // Invalid value
          ws_rear_left = 0.0f;
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_FORWARD) {  // forward value
          ws_rear_left = fabsf(ws_rear_left);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_BACKWARD) {  // backward value
          ws_rear_left = -fabsf(ws_rear_left);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_STANDSTILL) {  // standstill value
          ws_rear_left = 0;
        } else {
        }
      }
    }
  }

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_WS_REAR_LEFT;
  raw.data[idx].ws_rear_left.time_stamp_ = raw.raw_ts;
  raw.data[idx].ws_rear_left.ws_rear_left_ = ws_rear_left;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "wheel speed front right" << ws_rear_left;
  return 1;
}

int CANParser::ParseWSRearRight(const CANRawDataMap &raw_data_map,
                                CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_ws_rear_right);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  float ws_rear_right = 0;
  ws_rear_right = iter->second.value_.f32;

  // check direction
  iter = raw_data_map.find(CANRawDataType_ws_rear_right_dir);
  if ((iter != raw_data_map.end()) && iter->second.value_map_ != NULL) {
    CANValueMap::const_iterator iter_map;
    if (iter->second.value_type_ == CANRawDataValue_Signed) {
      iter_map = iter->second.value_map_->find(iter->second.value_.i32);
      if (iter_map != iter->second.value_map_->end()) {
        if (iter_map->second == WHEEL_DIR_TYPE_INVALID) {  // Invalid value
          ws_rear_right = 0.0f;
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_FORWARD) {  // forward value
          ws_rear_right = fabsf(ws_rear_right);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_BACKWARD) {  // backward value
          ws_rear_right = -fabsf(ws_rear_right);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_STANDSTILL) {  // standstill value
          ws_rear_right = 0;
        } else {
        }
      }
    } else {
      iter_map = iter->second.value_map_->find(iter->second.value_.u32);
      if (iter_map != iter->second.value_map_->end()) {
        if (iter_map->second == WHEEL_DIR_TYPE_INVALID) {  // Invalid value
          ws_rear_right = 0.0f;
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_FORWARD) {  // forward value
          ws_rear_right = fabsf(ws_rear_right);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_BACKWARD) {  // backward value
          ws_rear_right = -fabsf(ws_rear_right);
        } else if (iter_map->second ==
                   WHEEL_DIR_TYPE_STANDSTILL) {  // standstill value
          ws_rear_right = 0;
        } else {
        }
      }
    }
  }

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_WS_REAR_RIGHT;
  raw.data[idx].ws_rear_right.time_stamp_ = raw.raw_ts;
  raw.data[idx].ws_rear_right.ws_rear_right_ = ws_rear_right;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "wheel speed rear right" << ws_rear_right;
  return 1;
}

int CANParser::ParseWheelPulseFL(const CANRawDataMap &raw_data_map,
                                 CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_WheelPulseFL);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  int32_t WheelPulseFL = 0;
  WheelPulseFL = (int32_t)(iter->second.value_.f32);

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_WheelPulseFL;
  raw.data[idx].WheelPulseFL.time_stamp_ = raw.raw_ts;
  raw.data[idx].WheelPulseFL.whl_pule = WheelPulseFL;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "WheelPulseFL" << WheelPulseFL;
  return 1;
}
int CANParser::ParseWheelPulseFR(const CANRawDataMap &raw_data_map,
                                 CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_WheelPulseFR);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  int32_t WheelPulseFR = 0;
  WheelPulseFR = (int32_t)(iter->second.value_.f32);

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_WheelPulseFR;
  raw.data[idx].WheelPulseFR.time_stamp_ = raw.raw_ts;
  raw.data[idx].WheelPulseFR.whl_pule = WheelPulseFR;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "WheelPulseFR" << WheelPulseFR;
  return 1;
}
int CANParser::ParseWheelPulseRL(const CANRawDataMap &raw_data_map,
                                 CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_WheelPulseRL);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  int32_t WheelPulseRL = 0;
  WheelPulseRL = (int32_t)(iter->second.value_.f32);

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_WheelPulseRL;
  raw.data[idx].WheelPulseRL.time_stamp_ = raw.raw_ts;
  raw.data[idx].WheelPulseRL.whl_pule = WheelPulseRL;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "WheelPulseRL" << WheelPulseRL;
  return 1;
}
int CANParser::ParseWheelPulseRR(const CANRawDataMap &raw_data_map,
                                 CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_WheelPulseRR);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  int32_t WheelPulseRR = 0;
  WheelPulseRR = (int32_t)(iter->second.value_.f32);

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_WheelPulseRR;
  raw.data[idx].WheelPulseRR.time_stamp_ = raw.raw_ts;
  raw.data[idx].WheelPulseRR.whl_pule = WheelPulseRR;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "WheelPulseRR" << WheelPulseRR;
  return 1;
}
int CANParser::ParseWheelPulseFLDir(const CANRawDataMap &raw_data_map,
                                    CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_WheelPulseFLDir);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "WheelPulseFLDir";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /* LOG(ERROR) << str_buf << "value not found, "
         << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value not found, "
        << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int dir = iter_map->second;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_WheelPulseFLDir;
  raw.data[idx].WheelPulseFLDir.time_stamp_ = raw.raw_ts;
  raw.data[idx].WheelPulseFLDir.whl_pule_dir = dir;
  raw.info_count++;

  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << dir;
  return 1;
}
int CANParser::ParseWheelPulseFRDir(const CANRawDataMap &raw_data_map,
                                    CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_WheelPulseFRDir);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "WheelPulseFRDir";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value not found, "
        << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value not found, "
        << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int dir = iter_map->second;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_WheelPulseFRDir;
  raw.data[idx].WheelPulseFRDir.time_stamp_ = raw.raw_ts;
  raw.data[idx].WheelPulseFRDir.whl_pule_dir = dir;
  raw.info_count++;

  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << dir;
  return 1;
}
int CANParser::ParseWheelPulseRLDir(const CANRawDataMap &raw_data_map,
                                    CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_WheelPulseRLDir);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "WheelPulseRLDir";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value not found, "
        << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value not found, "
        << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int dir = iter_map->second;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_WheelPulseRLDir;
  raw.data[idx].WheelPulseRLDir.time_stamp_ = raw.raw_ts;
  raw.data[idx].WheelPulseRLDir.whl_pule_dir = dir;
  raw.info_count++;

  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << dir;
  return 1;
}
int CANParser::ParseWheelPulseRRDir(const CANRawDataMap &raw_data_map,
                                    CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_WheelPulseRRDir);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  std::string str_buf = "WheelPulseRRDir";

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /* LOG(ERROR) << str_buf << "value not found, "
         << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << str_buf << "value not found, "
        << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int dir = iter_map->second;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_WheelPulseRRDir;
  raw.data[idx].WheelPulseRRDir.time_stamp_ = raw.raw_ts;
  raw.data[idx].WheelPulseRRDir.whl_pule_dir = dir;
  raw.info_count++;

  // VLOG(LOG_VERBOSE_HIGHEST) << str_buf << dir;
  return 1;
}

int CANParser::ParseImuAccX(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw) {
  float imu_acc_x;
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_IMU_Acc_X);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  imu_acc_x = iter_raw->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_IMU_Acc_X;
  raw.data[idx].imu_acc_x.time_stamp_ = raw.raw_ts;
  raw.data[idx].imu_acc_x.imu_acc_x_ = imu_acc_x;
  raw.info_count++;

  return 1;
}

int CANParser::ParseImuAccY(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw) {
  float imu_acc_y;
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_IMU_Acc_Y);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  imu_acc_y = iter_raw->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_IMU_Acc_Y;
  raw.data[idx].imu_acc_y.time_stamp_ = raw.raw_ts;
  raw.data[idx].imu_acc_y.imu_acc_y_ = imu_acc_y;
  raw.info_count++;

  return 1;
}

int CANParser::ParseImuAccZ(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw) {
  float imu_acc_z;
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_IMU_Acc_Z);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  imu_acc_z = iter_raw->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_IMU_Acc_Z;
  raw.data[idx].imu_acc_z.time_stamp_ = raw.raw_ts;
  raw.data[idx].imu_acc_z.imu_acc_z_ = imu_acc_z;
  raw.info_count++;

  return 1;
}

int CANParser::ParseImuGyroX(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw) {
  float imu_gyro_x;
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_IMU_Gyro_X);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  imu_gyro_x = iter_raw->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_IMU_Gyro_X;
  raw.data[idx].imu_gyro_x.time_stamp_ = raw.raw_ts;
  raw.data[idx].imu_gyro_x.imu_gyro_x_ = imu_gyro_x;
  raw.info_count++;

  return 1;
}

int CANParser::ParseImuGyroY(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw) {
  float imu_gyro_y;
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_IMU_Gyro_Y);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  imu_gyro_y = iter_raw->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_IMU_Gyro_Y;
  raw.data[idx].imu_gyro_y.time_stamp_ = raw.raw_ts;
  raw.data[idx].imu_gyro_y.imu_gyro_y_ = imu_gyro_y;
  raw.info_count++;

  return 1;
}

int CANParser::ParseImuGyroZ(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw) {
  float imu_gyro_z;
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_IMU_Gyro_Z);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  imu_gyro_z = iter_raw->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_IMU_Gyro_Z;
  raw.data[idx].imu_gyro_z.time_stamp_ = raw.raw_ts;
  raw.data[idx].imu_gyro_z.imu_gyro_z_ = imu_gyro_z;
  raw.info_count++;

  return 1;
}

int CANParser::ParseCANYaw(const CANRawDataMap &raw_data_map,
                           CANFrameRaw &raw) {
  float yaw;
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_Yaw);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  yaw = iter_raw->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_Yaw;
  raw.data[idx].yaw.time_stamp_ = raw.raw_ts;
  raw.data[idx].yaw.yaw_ = yaw;
  raw.info_count++;

  return 1;
}

int CANParser::ParseOdoPosX(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw) {
  float odo_pos_x;
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_Odo_Pos_X);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  odo_pos_x = iter_raw->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_Odo_Pos_X;
  raw.data[idx].odo_pos_x.time_stamp_ = raw.raw_ts;
  raw.data[idx].odo_pos_x.odo_pos_x_ = odo_pos_x;
  raw.info_count++;

  return 1;
}

int CANParser::ParseOdoPosY(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw) {
  float odo_pos_y;
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_Odo_Pos_Y);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  odo_pos_y = iter_raw->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_Odo_Pos_Y;
  raw.data[idx].odo_pos_y.time_stamp_ = raw.raw_ts;
  raw.data[idx].odo_pos_y.odo_pos_y_ = odo_pos_y;
  raw.info_count++;

  return 1;
}

int CANParser::ParseOdoPosZ(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw) {
  float odo_pos_z;
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_Odo_Pos_Z);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  odo_pos_z = iter_raw->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_Odo_Pos_Z;
  raw.data[idx].odo_pos_z.time_stamp_ = raw.raw_ts;
  raw.data[idx].odo_pos_z.odo_pos_z_ = odo_pos_z;
  raw.info_count++;

  return 1;
}

int CANParser::ParseOdoSpeed(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw) {
  float odo_speed;
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_Odo_Speed);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  odo_speed = iter_raw->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_Odo_Speed;
  raw.data[idx].odo_speed.time_stamp_ = raw.raw_ts;
  raw.data[idx].odo_speed.odo_speed_ = odo_speed;
  raw.info_count++;

  return 1;
}

int CANParser::ParseOdoYawRate(const CANRawDataMap &raw_data_map,
                               CANFrameRaw &raw) {
  float odo_yawrate;
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_Odo_YawRate);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  odo_yawrate = iter_raw->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CANRawDataType_Odo_YawRate;
  raw.data[idx].odo_yawrate.time_stamp_ = raw.raw_ts;
  raw.data[idx].odo_yawrate.odo_yawrate_ = odo_yawrate;
  raw.info_count++;

  return 1;
}

int CANParser::ParseRoiStartX(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_RoiStartX);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  uint32_t roi_start_x = 0;
  roi_start_x = iter->second.value_.f32;

  iter = raw_data_map.find(CANRawDataType_RoiStartXValid);
  bool roi_start_x_valid = CANValidType_VALID;
  if ((iter != raw_data_map.end()) && iter->second.value_map_ != NULL) {
    CANValueMap::const_iterator iter_map;
    iter_map = iter->second.value_map_->find(iter->second.value_.u32);
    if (iter_map != iter->second.value_map_->end()) {
      if (iter_map->second == CANValidType_VALID) {
        roi_start_x_valid = CANValidType_VALID;
      } else {
        roi_start_x_valid = CANValidType_INVALID;
      }
    }
  }
  // VLOG(0) << "roi_start_x_valid = " << roi_start_x_valid;
  if (roi_start_x_valid == CANValidType_VALID) {
    int idx = raw.info_count;
    raw.type[idx] = CAN_FRAME_TYPE_ROI_START_X;
    raw.data[idx].roi_start_x.x_ = roi_start_x;
    raw.data[idx].roi_start_x.time_stamp_ = raw.raw_ts;
    raw.info_count++;
    // VLOG(LOG_VERBOSE_HIGHEST) << "roi_start_x " << roi_start_x;
  }
  // VLOG(0) << "roi_start_x " << roi_start_x;
  return 1;
}
int CANParser::ParseRoiStartY(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_RoiStartY);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  uint32_t roi_start_y = 0;
  roi_start_y = iter->second.value_.f32;

  iter = raw_data_map.find(CANRawDataType_RoiStartYValid);
  bool roi_start_y_valid = CANValidType_VALID;
  if ((iter != raw_data_map.end()) && iter->second.value_map_ != NULL) {
    CANValueMap::const_iterator iter_map;
    iter_map = iter->second.value_map_->find(iter->second.value_.u32);
    if (iter_map != iter->second.value_map_->end()) {
      if (iter_map->second == CANValidType_VALID) {
        roi_start_y_valid = CANValidType_VALID;
      } else {
        roi_start_y_valid = CANValidType_INVALID;
      }
    }
  }
  // VLOG(0) << "roi_start_y_valid " << roi_start_y_valid;
  if (roi_start_y_valid == CANValidType_VALID) {
    int idx = raw.info_count;
    raw.type[idx] = CAN_FRAME_TYPE_ROI_START_Y;
    raw.data[idx].roi_start_y.y_ = roi_start_y;
    raw.data[idx].roi_start_y.time_stamp_ = raw.raw_ts;
    raw.info_count++;
    // VLOG(LOG_VERBOSE_HIGHEST) << "roi_start_y " << roi_start_y;
  }
  // VLOG(0) << "roi_start_y " << roi_start_y;
  return 1;
}
int CANParser::ParseRoiWidth(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_RoiWidth);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  uint32_t roi_width = 0;
  roi_width = iter->second.value_.f32;

  iter = raw_data_map.find(CANRawDataType_RoiWidthValid);
  bool roi_width_valid = CANValidType_VALID;
  if ((iter != raw_data_map.end()) && iter->second.value_map_ != NULL) {
    CANValueMap::const_iterator iter_map;
    iter_map = iter->second.value_map_->find(iter->second.value_.u32);
    if (iter_map != iter->second.value_map_->end()) {
      if (iter_map->second == CANValidType_VALID) {
        roi_width_valid = CANValidType_VALID;
      } else {
        roi_width_valid = CANValidType_INVALID;
      }
    }
  }
  // VLOG(0) << "roi_width_valid " << roi_width_valid;
  if (roi_width_valid == CANValidType_VALID) {
    int idx = raw.info_count;
    raw.type[idx] = CAN_FRAME_TYPE_ROI_WIDTH;
    raw.data[idx].roi_width.width_ = roi_width;
    raw.data[idx].roi_width.time_stamp_ = raw.raw_ts;
    raw.info_count++;
    // VLOG(LOG_VERBOSE_HIGHEST) << "roi_width " << roi_width;
  }
  // VLOG(0) << "roi_width " << roi_width;
  return 1;
}
int CANParser::ParseRoiHeight(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_RoiHeight);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  uint32_t roi_height = 0;
  roi_height = iter->second.value_.f32;

  iter = raw_data_map.find(CANRawDataType_RoiHeightValid);
  bool roi_height_valid = CANValidType_VALID;
  if ((iter != raw_data_map.end()) && iter->second.value_map_ != NULL) {
    CANValueMap::const_iterator iter_map;
    iter_map = iter->second.value_map_->find(iter->second.value_.u32);
    if (iter_map != iter->second.value_map_->end()) {
      if (iter_map->second == CANValidType_VALID) {
        roi_height_valid = CANValidType_VALID;
      } else {
        roi_height_valid = CANValidType_INVALID;
      }
    }
  }
  // VLOG(0) << "roi_height_valid " << roi_height_valid;
  if (roi_height_valid == CANValidType_VALID) {
    int idx = raw.info_count;
    raw.type[idx] = CAN_FRAME_TYPE_ROI_HEIGHT;
    raw.data[idx].roi_height.height_ = roi_height;
    raw.data[idx].roi_height.time_stamp_ = raw.raw_ts;
    raw.info_count++;
    // VLOG(LOG_VERBOSE_HIGHEST) << "roi_height " << roi_height;
  }
  // VLOG(0) << "roi_height " << roi_height;
  return 1;
}

int CANParser::ParseRoiRollingCount(const CANRawDataMap &raw_data_map,
                                    CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_RoiRollingCount);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  uint32_t roi_rolling_count = 0;
  roi_rolling_count = iter->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_ROI_ROLLING_COUNT;
  raw.data[idx].roi_rolling_count.seq_num_ = roi_rolling_count;
  raw.data[idx].roi_rolling_count.time_stamp_ = raw.raw_ts;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "roi_rolling_count " << roi_rolling_count;
  // VLOG(0) << "roi_rolling_count " << roi_rolling_count;
  return 1;
}

int CANParser::ParseLowBeamStatus(const CANRawDataMap &raw_data_map,
                                  CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_LowBeamStatus);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << "low_beam_status " <<"value not found, "
                 << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << "low_beam_status " << "value not found, "
                 << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int low_beam_status = iter_map->second;

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_LOW_BEAM_STATUS;
  raw.data[idx].low_beam_status.low_beam_status_ = low_beam_status;
  raw.data[idx].low_beam_status.time_stamp_ = raw.raw_ts;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "low_beam_status " << low_beam_status;
  // VLOG(0) << "low_beam_status " << low_beam_status;
  return 1;
}

int CANParser::ParseHighBeamStatus(const CANRawDataMap &raw_data_map,
                                   CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_HighBeamStatus);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << "high_beam_status " <<"value not found, "
                 << iter_raw->second.value_.i32;*/
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      /*LOG(ERROR) << "high_beam_status " << "value not found, "
                 << iter_raw->second.value_.u32;*/
      return 0;
    }
  }

  int high_beam_status = iter_map->second;

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_HIGH_BEAM_STATUS;
  raw.data[idx].high_beam_status.high_beam_status_ = high_beam_status;
  raw.data[idx].high_beam_status.time_stamp_ = raw.raw_ts;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "high_beam_status " << high_beam_status;
  // VLOG(0) << "high_beam_status " << high_beam_status;
  return 1;
}

int CANParser::ParseAutoHeadlightStatus(const CANRawDataMap &raw_data_map,
                                        CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter_raw;
  iter_raw = raw_data_map.find(CANRawDataType_AutoHeadlightStatus);
  if (iter_raw == raw_data_map.end()) {
    return 0;
  }

  if (!iter_raw->second.value_map_) {
    return 0;
  }

  CANValueMap::const_iterator iter_map;
  if (iter_raw->second.value_type_ == CANRawDataValue_Signed) {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.i32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      // LOG(ERROR) << "auto_headlight_status " <<"value not found, "
      //<< iter_raw->second.value_.i32;
      return 0;
    }
  } else {
    iter_map = iter_raw->second.value_map_->find(iter_raw->second.value_.u32);
    if (iter_map == iter_raw->second.value_map_->end()) {
      // LOG(ERROR) << "auto_headlight_status " << "value not found, "
      //<< iter_raw->second.value_.u32;
      return 0;
    }
  }

  int auto_headlight_status = iter_map->second;

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_AUTO_HEADLIGHT_STATUS;
  raw.data[idx].auto_headlight_status.auto_headlight_status_ =
      auto_headlight_status;
  raw.data[idx].auto_headlight_status.time_stamp_ = raw.raw_ts;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "auto_headlight_status "
  // << auto_headlight_status;
  // VLOG(0) << "auto_headlight_status " << auto_headlight_status;
  return 1;
}
#if 1
int CANParser::Parseseconds(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_seconds);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  float tv_sec = 0;
  tv_sec = iter->second.value_.f32;
  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_seconds;
  raw.data[idx].CanTime_sec.tv_sec_ = tv_sec;
  raw.data[idx].CanTime_sec.time_stamp_ = raw.raw_ts;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "tv_sec " << tv_sec;
  return 1;
}

int CANParser::Parsenanoseconds(const CANRawDataMap &raw_data_map,
                                CANFrameRaw &raw) {
  CANRawDataMap::const_iterator iter;
  iter = raw_data_map.find(CANRawDataType_nanoseconds);
  if (iter == raw_data_map.end()) {
    return 0;
  }

  float tv_nsec = 0;
  tv_nsec = iter->second.value_.f32;

  int idx = raw.info_count;
  raw.type[idx] = CAN_FRAME_TYPE_nanoseconds;
  raw.data[idx].CanTime_nsec.tv_nsec_ = tv_nsec;
  raw.data[idx].CanTime_nsec.time_stamp_ = raw.raw_ts;
  raw.info_count++;
  // VLOG(LOG_VERBOSE_HIGHEST) << "tv_nsec " << tv_nsec;
  return 1;
}
#endif

int CANParser::ParseCanFrame(const int64_t &timestamp,
                             const struct can_frame *frame, CANFrameRaw *raw) {
  raw->raw = *frame;
  raw->raw_ts = timestamp;
  raw->info_count = 0;

  CANSettingMap::iterator setting_iter = frame_map_.find(frame->can_id);
  if (setting_iter == frame_map_.end()) {
    return 0;
  }

  CANRawDataMap raw_data_map;
  if (!ParseRawData(frame, &frame_map_, raw_data_map)) {
    return 0;
  }

  CANRawDataMap::iterator iter;
  for (iter = raw_data_map.begin(); iter != raw_data_map.end(); ++iter) {
    if (iter->first < 0 || iter->first >= CANRawDataType_COUNT) {
      continue;
    }
    if (can_cvt_func_[iter->first]) {
      CANFrameCvtFunc cvt = can_cvt_func_[iter->first];
      int count = 0;
      if (cvt != NULL) {
        count = (this->*cvt)(raw_data_map, *raw);
      }
    }
  }

  return raw->info_count;
}

RawInfo CANParser::FilterSpeed(int8_t &rst_flg, const RawInfo &rawinfo) {
  static float speed_val = FLT_EPSILON;
  static int speed_count = 0;
  RawInfo rawinfo_filted;
  if (rst_flg == 1) {
    speed_count = 0;
    speed_val = FLT_EPSILON;
    rst_flg = 0;
  }
  speed_count++;

  speed_val = (speed_val * (speed_count - 1) + rawinfo.sp.speed) / speed_count;
  rawinfo_filted.sp.speed = speed_val;
  rawinfo_filted.sp.time_stamp_ = rawinfo.sp.time_stamp_;
  return rawinfo_filted;
}

RawInfo CANParser::FilterTurnSignal(int8_t &rst_flg, const RawInfo &rawinfo) {
  RawInfo rawinfo_filted;

  if (rst_flg == 1) {
    rst_flg = 0;
  }
#if 1
  // delay (turn_signal_interval_ * 2) ms
  // when turn signal change from other status to TURN_SIGNAL_NONE
  // It is possible to use turning light signal as the turning indicator
  // which is always flash when it is active.
  if (first_ts_status_) {
    last_ts_status_ = rawinfo.ts;
    first_ts_status_ = false;
  } else {
    if (last_ts_status_.type == rawinfo.ts.type) {
      last_ts_status_ = rawinfo.ts;
    } else {
      if (rawinfo.ts.type != TURN_SIGNAL_NONE) {
        last_ts_status_ = rawinfo.ts;
      } else if (
#ifdef __QNX__
          std::llabs(rawinfo.ts.time_stamp_ - last_ts_status_.time_stamp_)
#else
          llabs(rawinfo.ts.time_stamp_ - last_ts_status_.time_stamp_)
#endif
          > turn_signal_interval_ * 2) {
        last_ts_status_ = rawinfo.ts;
      } else {
        // do nothing
      }
    }
  }
  rawinfo_filted.ts = last_ts_status_;

#else
  rawinfo_filted = rawinfo;
#endif
  return rawinfo_filted;
}

CANSender::CANSender() { can_socket_ = 0; }

CANSender::~CANSender() { can_socket_ = 0; }

bool CANSender::Init(const char *device, int can_protocol,
                     struct sockaddr_can *sockname) {
  // do nothing above context
#if defined(ADAS_FPGA)
  struct ifreq ifr;
  if (can_protocol == CAN_RAW) {
    struct sockaddr_can addr;

    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
      LOG(ERROR) << "create socket failed";
      return false;
    }

    VLOG(LOG_VERBOSE_HIGHEST) << "socket id =" << can_socket_;

    // strncpy(ifr.ifr_name, device, IFNAMSIZ - 1);
    // ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    // ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", device);
    /* determine the interface index */
    ioctl(can_socket_, SIOCGIFINDEX, &ifr);
    if (!ifr.ifr_ifindex) {
      LOG(ERROR) << "if_nametoindex";
      return false;
    }
    VLOG(0) << "index: " << ifr.ifr_ifindex;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    VLOG(LOG_VERBOSE_HIGHEST) << "ifindex = " << ifr.ifr_ifindex;

    setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
#ifdef __ANDROID__
      ALOGE("bind can device failed");
#endif
      LOG(ERROR) << "bind " << device << "Failed";
      return false;
    }
  } else if (can_protocol == CAN_J1939) {
    // create a socket
    can_socket_ = socket(PF_CAN, SOCK_DGRAM, CAN_J1939);
    VLOG(LOG_VERBOSE_HIGHEST) << "socket id =" << can_socket_;
    memset(&sockname_j1939_, 0, sizeof(sockname_j1939_));
    sockname_j1939_.can_family = AF_CAN;
    sockname_j1939_.can_addr.j1939.addr = sockname->can_addr.j1939.addr;

    sockname_j1939_.can_addr.j1939.name = sockname->can_addr.j1939.name;

    sockname_j1939_.can_addr.j1939.pgn = J1939_NO_PGN;

    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", device);
    /* determine the interface index */
    ioctl(can_socket_, SIOCGIFINDEX, &ifr);
    sockname_j1939_.can_ifindex = ifr.ifr_ifindex;

    // sockname_j1939_.can_ifindex = if_nametoindex(device);

    // bind socket
    if (bind(can_socket_, (struct sockaddr *)&sockname_j1939_,
             sizeof(sockname_j1939_)) < 0) {
#ifdef __ANDROID__
      ALOGE("bind can device failed");
#endif
      LOG(ERROR) << "bind " << device << " Failed";
      return false;
    }

    memset(&peername_j1939_, 0, sizeof(peername_j1939_));
    peername_j1939_.can_family = AF_CAN;
    peername_j1939_.can_addr.j1939.addr = J1939_NO_ADDR;
    peername_j1939_.can_addr.j1939.name = J1939_NO_NAME;
    peername_j1939_.can_addr.j1939.pgn = J1939_NO_PGN;
  } else if (can_protocol == CAN_SPI) {
#ifndef ADAS_XILINX
    char *frame = NULL;
    int frame_len = 0;
    headerType *header = NULL;
    int spi_frame_size = sizeof(headerType) + 1;
    std::vector<char> spi_frame(spi_frame_size, 0);

    int status = spi_dbus_init(5000000, 0);
    if (status < 0) {
      LOG(ERROR) << " spi device init fail";
    }

    status = spi_slave_dbus_init(0);
    if (status < 0) {
      LOG(ERROR) << " spi device slave init fail";
    }

    /*enable MCU CAN */
    spi_frame.resize(1 + sizeof(headerType));
    header = reinterpret_cast<headerType *>(spi_frame.data());
    header->type = MSG_CAN_SET_OPMODE;
    header->length = 1;
    spi_frame.back() = 1;
    frame = spi_frame.data();
    frame_len = static_cast<int>(spi_frame.size());
    int ret = spi_dbus_send_frame(frame, frame_len);
    if (ret < 0) {
      LOG(ERROR) << "send MCU CAN enble fail";
    }
#endif
  } else {
    // do nothing
  }
#endif
  return true;
}

void CANSender::Fini() {}

void CANSender::SendData(ADASOutputCAN *data, int can_protocol) {
#if defined(ADAS_FPGA)
  if (data == NULL) {
    return;
  }
  int vec_size = data->size();
  headerType *header = NULL;
  char i = 0;
  char j = 0;
  char *frame = NULL;
  int frame_len = 0;
  int spi_frame_size = sizeof(headerType) + vec_size * sizeof(struct can_frame);
  std::vector<char> spi_frame(spi_frame_size, 0);

  if (vec_size > 0) {
    header = reinterpret_cast<headerType *>(spi_frame.data());
    header->type = MSG_CAN_DATA_TRANSMIT;
    header->length = sizeof(struct can_frame) * vec_size;

    struct can_frame *can_txBuf = reinterpret_cast<struct can_frame *>(
        spi_frame.data() + sizeof(headerType));

    struct can_frame *tmp_frame;
    for (i = 0; i < vec_size; i++) {
      tmp_frame = can_txBuf + i;
      memset(tmp_frame, 0, sizeof(can_frame));
      tmp_frame->can_id = (*data)[i].can_id;
      tmp_frame->can_dlc = (*data)[i].can_dlc;
      memcpy(tmp_frame->data, (*data)[i].data, (*data)[i].can_dlc);
    }

    frame = spi_frame.data();
    frame_len = static_cast<int>(spi_frame.size());

#ifndef ADAS_XILINX
    int ret = spi_slave_dbus_send_frame(frame, frame_len);
    if (ret < 0) {
      LOG(ERROR) << "send can fail " << std::hex << ret;
    }
#endif
  }
#if 0
  std::vector<struct can_frame>::iterator iter;
  for (iter = data->begin(); iter != data->end(); iter++) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_id = iter->can_id;
    frame.can_dlc = iter->can_dlc;
    memcpy(frame.data, iter->data, iter->can_dlc);

    if (can_protocol == CAN_RAW) {
      int required_mtu = CAN_MTU;
      int nbytes = write(can_socket_, &frame, required_mtu);
      if (nbytes != required_mtu) {
        VLOG(LOG_VERBOSE_HIGHEST) << "write data to can failed, CAN ID: "
          << frame.can_id;
      } else {
        VLOG(LOG_VERBOSE_HIGHEST) << "write data to can, CAN ID: "
          << frame.can_id;
      }
    } else if (can_protocol == CAN_J1939) {
      CAN::CAN_ID_Type j1939_can_id;
      j1939_can_id._can_id_ = iter->can_id;
      if (j1939_can_id.j193_can_id_.prio_ >= 0) {
        int j1939_prio = j1939_can_id.j193_can_id_.prio_;
        if (setsockopt(can_socket_, SOL_CAN_J1939, SO_J1939_SEND_PRIO,
          &j1939_prio, sizeof(j1939_prio)) < 0) {
          VLOG(LOG_VERBOSE_HIGHEST) << "set j1939 priority failed ";
        }
      }
      if (j1939_can_id.j1939_pdux_.pf_ >= 240) {
        peername_j1939_.can_addr.j1939.addr = J1939_NO_ADDR;
        peername_j1939_.can_addr.j1939.pgn = j1939_can_id.j1939_pdu2_.pgn_;
        int nbytes = sendto(can_socket_, frame.data, iter->can_dlc,
          0, (struct sockaddr*)&peername_j1939_, sizeof(peername_j1939_));
        if (nbytes != iter->can_dlc) {
          VLOG(LOG_VERBOSE_HIGHEST) << "write data to can failed, J1939 pgn: "
            << peername_j1939_.can_addr.j1939.pgn;
        } else {
          VLOG(LOG_VERBOSE_HIGHEST) << "write data to can, J1939 pgn: "
            << peername_j1939_.can_addr.j1939.pgn;
        }
      } else {  // PDU1 format
        // pgn include DA
        peername_j1939_.can_addr.j1939.pgn = j1939_can_id.j193_can_id_.pgn_;
        peername_j1939_.can_addr.j1939.addr = j1939_can_id.j1939_pdu1_.da_;
        if (connect(can_socket_, (struct sockaddr*)&peername_j1939_,
          sizeof(peername_j1939_)) < 0) {
#ifdef __ANDROID__
          ALOGE("connect can device failed");
#endif
          LOG(ERROR) << "connect can socket Failed: " << can_socket_;
        }
        int nbytes = send(can_socket_, frame.data, iter->can_dlc, 0);
        if (nbytes != iter->can_dlc) {
          VLOG(LOG_VERBOSE_HIGHEST) << "write data to can failed, J1939 pgn: "
            << peername_j1939_.can_addr.j1939.pgn << " DA: "
            << peername_j1939_.can_addr.j1939.addr;
        } else {
          VLOG(LOG_VERBOSE_HIGHEST) << "write data to can, J1939 pgn: "
            << peername_j1939_.can_addr.j1939.pgn << " DA: "
            << peername_j1939_.can_addr.j1939.addr;
        }
      }
    } else {
      // do nothing
    }
  }
#endif
#endif
  return;
}

}  // end of namespace CAN
}  // end of namespace HobotADAS
