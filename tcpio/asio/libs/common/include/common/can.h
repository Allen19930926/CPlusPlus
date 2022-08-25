//
// Copyright 2016 Horizon Robotics.
//

#ifndef SRC_COMMON_CAN_H_
#define SRC_COMMON_CAN_H_

#include <stdint.h>

#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/timestamp.h"
#include "nlohmann/json.hpp"

#if defined(_WIN32) || defined(__MACH__) || defined(__QNX__) || \
    defined(__ANDROID__)
#define CAN_MAX_DLEN 8
#define CANFD_MAX_DLEN 64
struct can_frame {
  int can_id; /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  unsigned char can_dlc;
  unsigned char flags;
  unsigned char can_channel;
  unsigned char __res1;
  /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
  unsigned char data[CAN_MAX_DLEN];
  can_frame() : can_id(0), can_dlc(0) { memset(data, 0, sizeof(data)); }
};

struct canfd_frame {
  int can_id;                /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  unsigned char len;         /* frame payload length in byte */
  unsigned char flags;       /* additional flags for CAN FD */
  unsigned char can_channel; /* reserved / padding */
  unsigned char __res1;      /* reserved / padding */
  unsigned char data[CANFD_MAX_DLEN];
  canfd_frame() : can_id(0), len(0) { memset(data, 0, sizeof(data)); }
};

struct can_filter {
  int can_id;
  int can_mask;
};
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* externed frame format (EFF) */
struct sockaddr_can {
  uint32_t can_family;
  int can_ifindex;
  union {
    /* transport protocol class address information (e.g. ISOTP) */
    struct {
      uint32_t rx_id;
      uint32_t tx_id;
    } tp;
    /* J1939 address information */
    struct {
      uint64_t name;
      uint32_t pgn;
      uint8_t addr;
    } j1939;
  } can_addr;
};
#elif defined(__linux__) || defined(ADAS_FPGA)
#include <common/can-utils/linux/can.h>
#include <common/can-utils/linux/can/raw.h>
#endif

typedef enum WHEEL_DIR_TYPE_ {
  WHEEL_DIR_TYPE_FORWARD = 0,
  WHEEL_DIR_TYPE_BACKWARD = 1,
  WHEEL_DIR_TYPE_STANDSTILL = 2,
  WHEEL_DIR_TYPE_INVALID = 3
} WHEEL_DIR_TYPE;

namespace HobotADAS {
class HobotADASContext;
// class Frame;

typedef std::vector<struct can_frame> ADASOutputCAN;

namespace CAN {

#define MAX_CAN_RAW_OUTPUT 64

enum CANRawEndianType {
  CANRawEndianType_Little = 0,
  CANRawEndianType_Big = 1,
};

// can raw data type
enum CANRawDataValueType {
  CANRawDataValue_Unsigned = 0,
  CANRawDataValue_Signed = 1,
  CANRawDataValue_Float = 2,
  CANRawDataValue_Double = 3,
  CANRawDataValue_Enum = 4,
};

// raw data type for single can frame

enum CANRawDataType {
  CANRawDataType_Gear = 0,
  CANRawDataType_WheelAngle = 1,
  CANRawDataType_Speed = 2,
  CANRawDataType_TurnSignal = 3,
  CANRawDataType_Acc = 4,
  CANRawDataType_YawRate = 5,
  CANRawDataType_YawRateOffset = 6,
  CANRawDataType_lateral_acc = 7,
  CANRawDataType_brake_pedal_position = 8,
  CANRawDataType_acc_pedal_position = 9,
  CANRawDataType_power_mode = 10,
  CANRawDataType_hazard_light = 11,
  CANRawDataType_head_wiper = 12,
  CANRawDataType_outside_temperatrue = 13,
  CANRawDataType_light_level = 14,
  CANRawDataType_hmw_enable = 15,
  CANRawDataType_fcw_enable = 16,
  CANRawDataType_ldw_enable = 17,
  CANRawDataType_pcw_enable = 18,
  CANRawDataType_ufcw_enable = 19,
  CANRawDataType_ws_front_left = 20,
  CANRawDataType_ws_front_right = 21,
  CANRawDataType_ws_rear_left = 22,
  CANRawDataType_ws_rear_right = 23,
  CANRawDataType_YawRateDirection = 24,
  CANRawDataType_WheelAngleDirection = 25,
  CANRawDataType_ws_front_left_valid = 26,
  CANRawDataType_ws_front_right_valid = 27,
  CANRawDataType_ws_rear_left_valid = 28,
  CANRawDataType_ws_rear_right_valid = 29,
  CANRawDataType_ws_front_left_dir = 30,
  CANRawDataType_ws_front_right_dir = 31,
  CANRawDataType_ws_rear_left_dir = 32,
  CANRawDataType_ws_rear_right_dir = 33,
  CANRawDataType_WheelAngleValid = 34,
  CANRawDataType_YawRateValid = 35,
  CANRawDataType_WheelPulseFL = 36,
  CANRawDataType_WheelPulseFR = 37,
  CANRawDataType_WheelPulseRL = 38,
  CANRawDataType_WheelPulseRR = 39,
  CANRawDataType_WheelPulseFLDir = 40,
  CANRawDataType_WheelPulseFRDir = 41,
  CANRawDataType_WheelPulseRLDir = 42,
  CANRawDataType_WheelPulseRRDir = 43,
  CANRawDataType_WheelPulseFLValid = 44,
  CANRawDataType_WheelPulseFRValid = 45,
  CANRawDataType_WheelPulseRLValid = 46,
  CANRawDataType_WheelPulseRRValid = 47,
  CANRawDataType_seconds = 48,
  CANRawDataType_nanoseconds = 49,
  CANRawDataType_VehicleSpeedValid = 50,
  CANRawDataType_IMU_Acc_X = 51,
  CANRawDataType_IMU_Acc_Y = 52,
  CANRawDataType_IMU_Acc_Z = 53,
  CANRawDataType_IMU_Gyro_X = 54,
  CANRawDataType_IMU_Gyro_Y = 55,
  CANRawDataType_IMU_Gyro_Z = 56,
  CANRawDataType_Yaw = 57,
  CANRawDataType_Odo_Pos_X = 58,
  CANRawDataType_Odo_Pos_Y = 59,
  CANRawDataType_Odo_Pos_Z = 60,
  CANRawDataType_Odo_Speed = 61,
  CANRawDataType_Odo_YawRate = 62,
  CANRawDataType_LeftTurnSignal = 63,
  CANRawDataType_RightTurnSignal = 64,
  CANRawDataType_RoiStartX = 65,
  CANRawDataType_RoiStartY = 66,
  CANRawDataType_RoiWidth = 67,
  CANRawDataType_RoiHeight = 68,
  CANRawDataType_RoiStartXValid = 69,
  CANRawDataType_RoiStartYValid = 70,
  CANRawDataType_RoiWidthValid = 71,
  CANRawDataType_RoiHeightValid = 72,
  CANRawDataType_RoiRollingCount = 73,
  CANRawDataType_LowBeamStatus = 74,
  CANRawDataType_HighBeamStatus = 75,
  CANRawDataType_AutoHeadlightStatus = 76,
  CANRawDataType_UDLCLeftStatus = 77,
  CANRawDataType_UDLCRightStatus = 78,
  CANRawDataType_COUNT = 79
};

enum CANSignalDiagType {
  CANSignal_Diagnosis_Speed = 0,
  CANSignal_Diagnosis_Yawrate = 1,
  CANSignal_Diagnosis_Speed_Timeout = 2,
  CANSignal_Diagnosis_Yawrate_Timeout = 3,
  CANSignal_Diagnosis_Num = 4
};

enum CANSignalDiagStatus {
  CANSignal_Diagnosis_Status_Init = 0,
  CANSignal_Diagnosis_Status_Pass = 1,
  CANSignal_Diagnosis_Status_Fail = 2,
  CANSignal_Diagnosis_Status_Stable = 3,
  CANSignal_Diagnosis_Status_None = 4
};

typedef std::unordered_map<int, std::string> CANValueDescrip;
typedef std::unordered_map<int, int> CANValueMap;

struct CANSignalDiagInfo {
  CANSignalDiagStatus pre_;
  CANSignalDiagStatus cur_;
};

typedef struct CANSignalStatusType_ {
  int64_t last_timestamp_;
  bool timeout_flag_;
} CANSignalStatusType;

typedef struct CANSignalStsCfgType_ {
  int64_t timeout_time_;
  bool timeout_monitor_;
} CANSignalStsCfgType;

struct CANSignal_st {
  uint8_t singal_;
  uint8_t start_bit_;
  uint8_t bit_length_;
  uint8_t value_type_;
  uint8_t map_size_;
  uint8_t unused[3];
  float scale_;
  float offset_;
  std::vector<uint8_t> value_map_;
};

struct CANFilter_st {
  int can_id_;
  uint8_t endian_;
  uint8_t signal_num_;
  uint16_t cycle_;
  std::vector<CANSignal_st> signals_;
};

class CANSetting {
 public:
  CANSetting() {
    can_id_ = -1;
    start_bit_ = 0;
    bit_length_ = 0;
    raw_type_ = -1;
    endian_ = CANRawEndianType_Little;
    value_type_ = CANRawDataValue_Unsigned;
    scale_ = 1.0;
    offset_ = 0.0;
  }

  int can_id_;
  int raw_type_;
  int endian_;
  int start_bit_;
  int bit_length_;
  int value_type_;
  float scale_;
  float offset_;
  float min_;
  float max_;
  CANValueDescrip value_table_;  // description for each value
  CANValueMap value_Map_;  // map enum value to CANFrame value, e.g. geary type

  void DescriptionToValueMap();
};

struct CANRawData {
  CANRawData() { value_map_ = NULL; }
  union {
    uint32_t u32;
    int32_t i32;
    float f32;
    double f64;
  } value_;
  int value_type_;
  const CANValueMap *value_map_;
};

typedef std::unordered_map<int, CANRawData> CANRawDataMap;

// output type for CANFrame
typedef enum CANFrameType_ {
  CAN_FRAME_TYPE_GEAR = 0,
  CAN_FRAME_TYPE_WHEEL_ANGLE = 1,
  CAN_FRAME_TYPE_SPEED = 2,
  CAN_FRAME_TYPE_TURN_SIGNAL = 3,
  CAN_FRAME_TYPE_ACC = 4,
  CAN_FRAME_TYPE_YAW_RATE = 5,
  CAN_FRAME_TYPE_YAW_RATE_OFFSET = 6,
  CAN_FRAME_TYPE_LATERAL_ACC = 7,
  CAN_FRAME_TYPE_BRAKE_PEDAL_POSITION = 8,
  CAN_FRAME_TYPE_ACC_PEDAL_POSITION = 9,
  CAN_FRAME_TYPE_POWER_MODE = 10,
  CAN_FRAME_TYPE_HAZARD_LIGHT = 11,
  CAN_FRAME_TYPE_HEAD_WIPER = 12,
  CAN_FRAME_TYPE_OUTSIDE_TEMPERATRUE = 13,
  CAN_FRAME_TYPE_LIGHT_LEVEL = 14,
  CAN_FRAME_TYPE_HMW_ENABLE = 15,
  CAN_FRAME_TYPE_FCW_ENABLE = 16,
  CAN_FRAME_TYPE_LDW_ENABLE = 17,
  CAN_FRAME_TYPE_PCW_ENABLE = 18,
  CAN_FRAME_TYPE_UFCW_ENABLE = 19,
  CAN_FRAME_TYPE_WS_FRONT_LEFT = 20,
  CAN_FRAME_TYPE_WS_FRONT_RIGHT = 21,
  CAN_FRAME_TYPE_WS_REAR_LEFT = 22,
  CAN_FRAME_TYPE_WS_REAR_RIGHT = 23,
  CAN_FRAME_TYPE_YAW_RATE_DIRECTION = 24,
  CAN_FRAME_TYPE_WHEEL_ANGLE_DIRECTION = 25,
  CAN_FRAME_TYPE_WS_FRONT_LEFT_VALID = 26,
  CAN_FRAME_TYPE_WS_FRONT_RIGHT_VALID = 27,
  CAN_FRAME_TYPE_WS_REAR_LEFT_VALID = 28,
  CAN_FRAME_TYPE_WS_REAR_RIGHT_VALID = 29,
  CAN_FRAME_TYPE_WS_FRONT_LEFT_DIR = 30,
  CAN_FRAME_TYPE_WS_FRONT_RIGHT_DIR = 31,
  CAN_FRAME_TYPE_WS_REAR_LEFT_DIR = 32,
  CAN_FRAME_TYPE_WS_REAR_RIGHT_DIR = 33,
  CAN_FRAME_TYPE_WHEELANGLEVALID = 34,
  CAN_FRAME_TYPE_YAWRATEVALID = 35,
  CAN_FRAME_TYPE_WHEELPULSEFL = 36,
  CAN_FRAME_TYPE_WHEELPULSEFR = 37,
  CAN_FRAME_TYPE_WHEELPULSERL = 38,
  CAN_FRAME_TYPE_WHEELPULSERR = 39,
  CAN_FRAME_TYPE_WHEELPULSEFLDIR = 40,
  CAN_FRAME_TYPE_WHEELPULSEFRDIR = 41,
  CAN_FRAME_TYPE_WHEELPULSERLDIR = 42,
  CAN_FRAME_TYPE_WHEELPULSERRDIR = 43,
  CAN_FRAME_TYPE_WHEELPULSEFLVALID = 44,
  CAN_FRAME_TYPE_WHEELPULSEFRVALID = 45,
  CAN_FRAME_TYPE_WHEELPULSERLVALID = 46,
  CAN_FRAME_TYPE_WHEELPULSERRVALID = 47,
  CAN_FRAME_TYPE_seconds = 48,
  CAN_FRAME_TYPE_nanoseconds = 49,
  CAN_FRAME_TYPE_VehicleSpeedValid = 50,
  CAN_FRAME_TYPE_IMU_ACC_X = 51,
  CAN_FRAME_TYPE_IMU_ACC_Y = 52,
  CAN_FRAME_TYPE_IMU_ACC_Z = 53,
  CAN_FRAME_TYPE_IMU_GYRO_X = 54,
  CAN_FRAME_TYPE_IMU_GYRO_Y = 55,
  CAN_FRAME_TYPE_IMU_GYRO_Z = 56,
  CAN_FRAME_TYPE_YAW = 57,
  CAN_FRAME_TYPE_ODO_POS_X = 58,
  CAN_FRAME_TYPE_ODO_POS_Y = 59,
  CAN_FRAME_TYPE_ODO_POS_Z = 60,
  CAN_FRAME_TYPE_ODO_SPEED = 61,
  CAN_FRAME_TYPE_ODO_YAWRATE = 62,
  CAN_FRAME_TYPE_LEFT_TURN_SIGNAL = 63,
  CAN_FRAME_TYPE_RIGHT_TURN_SIGNAL = 64,
  CAN_FRAME_TYPE_ROI_START_X = 65,
  CAN_FRAME_TYPE_ROI_START_Y = 66,
  CAN_FRAME_TYPE_ROI_WIDTH = 67,
  CAN_FRAME_TYPE_ROI_HEIGHT = 68,
  CAN_FRAME_TYPE_ROI_START_X_VALID = 69,
  CAN_FRAME_TYPE_ROI_START_Y_VALID = 70,
  CAN_FRAME_TYPE_ROI_WIDTH_VALID = 71,
  CAN_FRAME_TYPE_ROI_HEIGHT_VALID = 72,
  CAN_FRAME_TYPE_ROI_ROLLING_COUNT = 73,
  CAN_FRAME_TYPE_LOW_BEAM_STATUS = 74,
  CAN_FRAME_TYPE_HIGH_BEAM_STATUS = 75,
  CAN_FRAME_TYPE_AUTO_HEADLIGHT_STATUS = 76,
  CAN_FRAME_TYPE_UDLCLeftStatus = 77,
  CAN_FRAME_TYPE_UDLCRightStatus = 78,
  CAN_FRAME_TYPE_PROCESS_COUNT = 79,
  CAN_FRAME_TYPE_EXTRA = 80,  // for verbose mode
  CAN_FRAME_TYPE_COUNT = 81
} CANFrameType;

typedef enum GearType_ {
  GEAR_TYPE_N = 0,
  GEAR_TYPE_P = 1,
  GEAR_TYPE_R = 2,
  GEAR_TYPE_D = 3,
  GEAR_TYPE_NO_OP = 4,
  GEAR_TYPE_COUNT = 5
} GearType;

typedef enum LightType_ {
  LIGHT_OFF = 0,
  LIGHT_ON = 1,
  LIGHT_INVALID = 2
} LightType;

typedef enum TurnSignalType_ {
  TURN_SIGNAL_NONE = 0,
  TURN_SIGNAL_LEFT = 1,
  TURN_SIGNAL_RIGHT = 2,
  TURN_SIGNAL_DOUBLE = 3,
  TURN_SIGNAL_COUNT = 4
} TurnSignalType;

typedef enum UDLCStatusType_ {
  UDLC_Not_Ready = 0,
  UDLC_Ready = 1,
  UDLC_Active_Wait = 2,
  UDLC_Active_Delay = 3,
  UDLC_ChangeActive = 4,
  UDLC_Not_Active_by_Obj = 5,
  UDLC_Invalid = 15
} UDLCStatusType;

typedef enum PowerModeType_ {
  PowerMode_OFF = 0,
  PowerMode_ACC = 1,
  PowerMode_ON = 2,
  PowerMode_CRANK = 3
} PowerModeType;

typedef enum HazardLightType_ {
  HazardLight_OFF = 0,
  HazardLight_ON = 1,
} HazardLightType;

typedef enum HeadWiperType_ {
  HeadWiper_OFF = 0,
  HeadWiper_ACTIVE = 1,
  HeadWiper_Reserved = 2
} HeadWiperType;

typedef enum ActiveType_ { INACTIVE = 0, ACTIVE = 1, INVALID = 2 } ActiveType;

typedef enum LightLevelType_ {
  LightLevel_UNKNOWN = 0,
  LightLevel_NIGHT = 1,
  LightLevel_DAY = 2
} LightLevelType;

typedef enum WarnEnableType_ {
  WarnEnable_DISABLE = 0,
  WarnEnable_ENABLE = 1,
  WarnEnable_INHIBIT = 2,
  WarnEnable_RESERVED = 3
} WarnEnableType;

typedef enum CANValidType_ {
  CANValidType_INVALID = 0,
  CANValidType_VALID = 1
} CANValidType;

typedef std::unordered_map<int, std::vector<CANSetting> > CANSettingMap;

typedef struct VehSpeed_ {
  TimeStamp time_stamp_;
  float speed;
} VehSpeed;

typedef struct WheelAngle_ {
  TimeStamp time_stamp_;
  float angle;
} WheelAngle;

typedef struct CANWheelAngleDirection_ {
  TimeStamp time_stamp_;
  int type;
} CANWheelAngleDirection;

typedef struct Gear_ {
  TimeStamp time_stamp_;
  int type;
} Gear;

typedef struct TurnSignal_ {
  TimeStamp time_stamp_;
  int type;
} TurnSignal;

typedef struct LeftTurnSignal_ {
  TimeStamp time_stamp_;
  int type;
} LeftTurnSignal;

typedef struct RightTurnSignal_ {
  TimeStamp time_stamp_;
  int type;
} RightTurnSignal;

typedef struct UDLCLeftStatus_ {
  TimeStamp time_stamp_;
  int type;
} UDLCLeftStatus;

typedef struct UDLCRightStatus_ {
  TimeStamp time_stamp_;
  int type;
} UDLCRightStatus;

typedef struct CANAcc_ {
  TimeStamp time_stamp_;
  float acc_;
} CANAcc;

typedef struct CANYawRate_ {
  TimeStamp time_stamp_;
  float yaw_rate_;
} CANYawRate;

typedef struct CANYawRateOffset_ {
  TimeStamp time_stamp_;
  float yaw_rate_offset_;
} CANYawRateOffset;

typedef struct CANYawRateDirection_ {
  TimeStamp time_stamp_;
  int type;
} CANYawRateDirection;

typedef struct CANLateralAcc_ {
  TimeStamp time_stamp_;
  float lateral_acc_;
} CANLateralAcc;

typedef struct BrakePedalPosition_ {
  TimeStamp time_stamp_;
  float brake_pedal_position_;
} BrakePedalPosition;

typedef struct AccPedalPosition_ {
  TimeStamp time_stamp_;
  float acc_pedal_position_;
} AccPedalPosition;

typedef struct CANPowerMode_ {
  TimeStamp time_stamp_;
  int power_mode_;
} CANPowerMode;

typedef struct HazardLight_ {
  TimeStamp time_stamp_;
  bool hazard_light_;
} HazardLight;

typedef struct HeadWiper_ {
  TimeStamp time_stamp_;
  int head_wiper_;
} HeadWiper;

typedef struct OutsideTemperatrue_ {
  TimeStamp time_stamp_;
  float outside_temperatrue_;
} OutsideTemperatrue;

typedef struct CANLightLevel_ {
  TimeStamp time_stamp_;
  int light_level_;
} CANLightLevel;

typedef struct HMWEnable_ {
  TimeStamp time_stamp_;
  int hmw_enable_;
} HMWEnable;

typedef struct FCWEnable_ {
  TimeStamp time_stamp_;
  int fcw_enable_;
} FCWEnable;

typedef struct LDWEnable_ {
  TimeStamp time_stamp_;
  int ldw_enable_;
} LDWEnable;

typedef struct PCWEnable_ {
  TimeStamp time_stamp_;
  int pcw_enable_;
} PCWEnable;

typedef struct UFCWEnable_ {
  TimeStamp time_stamp_;
  int ufcw_enable_;
} UFCWEnable;

typedef struct WSFrontLeft_ {
  TimeStamp time_stamp_;
  float ws_front_left_;
} WSFrontLeft;

typedef struct WSFrontRight_ {
  TimeStamp time_stamp_;
  float ws_front_right_;
} WSFrontRight;

typedef struct WSRearLeft_ {
  TimeStamp time_stamp_;
  float ws_rear_left_;
} WSRearLeft;

typedef struct WSRearRight_ {
  TimeStamp time_stamp_;
  float ws_rear_right_;
} WSFRearRight;

typedef struct WSValid_ {
  TimeStamp time_stamp_;
  bool ws_valid;
} WSValid;

typedef struct WSDir_ {
  TimeStamp time_stamp_;
  int ws_dir;
} WSDir;

typedef struct WAValid_ {
  TimeStamp time_stamp_;
  bool wa_valid;
} WAValid;

typedef struct YAWRATEValid_ {
  TimeStamp time_stamp_;
  bool yawrate_valid;
} YAWRATEValid;

typedef struct WHL_PULE_ {
  TimeStamp time_stamp_;
  int whl_pule;
} WHL_PULE;

typedef struct WHL_PULE_Valid_ {
  TimeStamp time_stamp_;
  bool whl_pule_valid;
} WHL_PULE_Valid;

typedef struct WHL_PULE_Dir_ {
  TimeStamp time_stamp_;
  int whl_pule_dir;
} WHL_PULE_Dir;

typedef struct RawInfoFloat_ {
  TimeStamp time_stamp_;
  float data_value_;
} RawInfoFloat;

typedef struct RawInfoInt_ {
  TimeStamp time_stamp_;
  int data_value_;
} RawInfoInt;

typedef struct RawInfoIBool_ {
  TimeStamp time_stamp_;
  bool data_value_;
} RawInfoBool;

typedef struct Sec_CanTime_ {
  TimeStamp time_stamp_;
  float tv_sec_;
} Sec_CanTme;

typedef struct Nsec_CanTime_ {
  TimeStamp time_stamp_;
  float tv_nsec_;
} Nsec_CanTme;

typedef struct Veh_Speed_Valid_t {
  TimeStamp time_stamp_;
  bool speed_valid;
} Veh_Speed_Valid;

typedef struct IMU_Acc_X_t {
  TimeStamp time_stamp_;
  float imu_acc_x_;
} Imu_Acc_X;

typedef struct IMU_Acc_Y_t {
  TimeStamp time_stamp_;
  float imu_acc_y_;
} Imu_Acc_Y;

typedef struct IMU_Acc_Z_t {
  TimeStamp time_stamp_;
  float imu_acc_z_;
} Imu_Acc_Z;

typedef struct IMU_Gyro_X_t {
  TimeStamp time_stamp_;
  float imu_gyro_x_;
} Imu_Gyro_X;

typedef struct IMU_Gyro_Y_t {
  TimeStamp time_stamp_;
  float imu_gyro_y_;
} Imu_Gyro_Y;

typedef struct IMU_Gyro_Z_t {
  TimeStamp time_stamp_;
  float imu_gyro_z_;
} Imu_Gyro_Z;

typedef struct CANYaw_ {
  TimeStamp time_stamp_;
  float yaw_;
} CANYaw;

typedef struct Odo_Pos_X_ {
  TimeStamp time_stamp_;
  float odo_pos_x_;
} Odo_Pos_X;

typedef struct Odo_Pos_Y_ {
  TimeStamp time_stamp_;
  float odo_pos_y_;
} Odo_Pos_Y;

typedef struct Odo_Pos_Z_ {
  TimeStamp time_stamp_;
  float odo_pos_z_;
} Odo_Pos_Z;

typedef struct Odo_Speed_ {
  TimeStamp time_stamp_;
  float odo_speed_;
} Odo_Speed;

typedef struct Odo_YawRate_ {
  TimeStamp time_stamp_;
  float odo_yawrate_;
} Odo_YawRate;

typedef struct RoiStartX_ {
  TimeStamp time_stamp_;
  uint32_t x_;
} RoiStartX;

typedef struct RoiStartY_ {
  TimeStamp time_stamp_;
  uint32_t y_;
} RoiStartY;

typedef struct RoiWidth_ {
  TimeStamp time_stamp_;
  uint32_t width_;
} RoiWidth;

typedef struct RoiHeight_ {
  TimeStamp time_stamp_;
  uint32_t height_;
} RoiHeight;

typedef struct RoiRollingCount_ {
  TimeStamp time_stamp_;
  uint32_t seq_num_;
} RoiRollingCount;

typedef struct LowBeamStatus_ {
  TimeStamp time_stamp_;
  uint32_t low_beam_status_;
} LowBeamStatus;

typedef struct HighBeamStatus_ {
  TimeStamp time_stamp_;
  uint32_t high_beam_status_;
} HighBeamStatus;

typedef struct AutoHeadlightStatus_ {
  TimeStamp time_stamp_;
  uint32_t auto_headlight_status_;
} AutoHeadlightStatus;

union RawInfo {
  TurnSignal ts;
  Gear gt;
  VehSpeed sp;
  WheelAngle wa;
  CANAcc acc;
  CANYawRate yaw_rate;
  CANYawRateOffset yaw_rate_offset;
  CANLateralAcc lateral_acc;
  BrakePedalPosition brake_pedal_position;
  AccPedalPosition acc_pedal_position;
  CANPowerMode power_mode;
  HazardLight hazard_light;
  HeadWiper head_wiper;
  OutsideTemperatrue outside_temperatrue;
  CANLightLevel light_level;
  HMWEnable hmw_enable;
  FCWEnable fcw_enable;
  LDWEnable ldw_enable;
  PCWEnable pcw_enable;
  UFCWEnable ufcw_enable;
  WSFrontLeft ws_front_left;
  WSFrontRight ws_front_right;
  WSRearLeft ws_rear_left;
  WSFRearRight ws_rear_right;
  RawInfoFloat general_float;
  RawInfoInt general_int;
  RawInfoBool general_bool;
  CANWheelAngleDirection wa_direction;
  CANYawRateDirection yaw_rate_direction;
  WSValid ws_front_left_valid;
  WSValid ws_front_right_valid;
  WSValid ws_rear_left_valid;
  WSValid ws_rear_right_valid;
  WSDir ws_front_left_dir;
  WSDir ws_front_right_dir;
  WSDir ws_rear_left_dir;
  WSDir ws_rear_right_dir;
  WAValid WheelAngleValid;
  YAWRATEValid YawRateValid;
  WHL_PULE WheelPulseFL;
  WHL_PULE WheelPulseFR;
  WHL_PULE WheelPulseRL;
  WHL_PULE WheelPulseRR;
  WHL_PULE_Valid WheelPulseFLValid;
  WHL_PULE_Valid WheelPulseFRValid;
  WHL_PULE_Valid WheelPulseRLValid;
  WHL_PULE_Valid WheelPulseRRValid;
  WHL_PULE_Dir WheelPulseFLDir;
  WHL_PULE_Dir WheelPulseFRDir;
  WHL_PULE_Dir WheelPulseRLDir;
  WHL_PULE_Dir WheelPulseRRDir;
  Sec_CanTme CanTime_sec;
  Nsec_CanTme CanTime_nsec;
  Veh_Speed_Valid VehicleSpeedValid;
  Imu_Acc_X imu_acc_x;
  Imu_Acc_Y imu_acc_y;
  Imu_Acc_Z imu_acc_z;
  Imu_Gyro_X imu_gyro_x;
  Imu_Gyro_Y imu_gyro_y;
  Imu_Gyro_Z imu_gyro_z;
  CANYaw yaw;
  Odo_Pos_X odo_pos_x;
  Odo_Pos_Y odo_pos_y;
  Odo_Pos_Z odo_pos_z;
  Odo_Speed odo_speed;
  Odo_YawRate odo_yawrate;
  LeftTurnSignal left_turn_signal;
  RightTurnSignal right_turn_signal;
  RoiStartX roi_start_x;
  RoiStartY roi_start_y;
  RoiWidth roi_width;
  RoiHeight roi_height;
  RoiRollingCount roi_rolling_count;
  LowBeamStatus low_beam_status;
  HighBeamStatus high_beam_status;
  AutoHeadlightStatus auto_headlight_status;
  UDLCLeftStatus udlc_left_status;
  UDLCRightStatus udlc_right_status;
};

typedef struct CANFrameRaw_ {
  uint8_t info_count;
  uint8_t type[MAX_CAN_RAW_OUTPUT];
  RawInfo data[MAX_CAN_RAW_OUTPUT];
  struct can_frame raw;
  TimeStamp raw_ts;
} CANFrameRaw;

typedef struct CANFDFrameRaw_ {
  uint8_t info_count;
  uint8_t type[MAX_CAN_RAW_OUTPUT];
  RawInfo data[MAX_CAN_RAW_OUTPUT];
  struct canfd_frame raw;
  TimeStamp raw_ts;
} CANFDFrameRaw;

class CANFrame {
 public:
  CANFrame();
  ~CANFrame();

  std::vector<RawInfo> data_;
};

class CANParser {
 public:
  CANParser();
  ~CANParser();

  bool Init(std::string path);

  void Fini();

  int GetCanFilter(struct can_filter *filters, int buf_count);

  bool Feed(struct can_frame *frame, TimeStamp ts);
  bool AddCanRaw(struct can_frame *frame, TimeStamp ts);
  bool Feed(struct canfd_frame *frame, TimeStamp ts);
  bool Summarize(TimeStamp time_stamp, CANFrame &can_frame,
                 std::vector<CAN::CANFrameRaw> &can_frame_raw,
                 std::vector<CAN::CANFDFrameRaw> &canfd_frame_raw);
  bool verbose_mode_;
  int max_can_frame_;
  const CANFrameRaw &GetLastCanFrameRaw() { return last_can_frame_raw_; }
  TimeStamp GetFrontCanFrameRawTs() {
    std::lock_guard<std::mutex> lck(can_mutex_);
    if (can_list_.size() > 0) {
      return can_list_.front().raw_ts;
    } else {
      return 0;
    }
  }
  TimeStamp GetLastCanFrameRawTs() {
    std::lock_guard<std::mutex> lck(can_mutex_);
    if (can_list_.size() > 0) {
      return can_list_.back().raw_ts;
    } else {
      return 0;
    }
  }
  bool GetCANFilterSetting(std::string path,
                           std::vector<struct CANFilter_st> &filter);

  std::vector<CANSignalDiagInfo> can_diag_state_;
  std::vector<CANSignalStatusType> can_signal_status_;
  std::vector<CANSignalStsCfgType> can_signal_timeout_config_;

 protected:
  std::list<CANFrameRaw> can_list_;
  std::list<CANFDFrameRaw> canfd_list_;
  std::mutex can_mutex_;
  std::mutex canfd_mutex_;
  std::condition_variable can_cond_;
  std::vector<bool> check_signal_can_;
  CANSettingMap frame_map_;

  TurnSignal last_ts_status_;
  bool first_ts_status_;
  int turn_signal_interval_;
  // prefetch N ms can raw data in front of image ts.
  int prefetch_time_;

  std::vector<CANFrameRaw> last_raw_;

  CANFrame CANFrameSummary(const TimeStamp &target_ts,
                           const CANFrameRaw *can_frame_raw, size_t can_count);
  bool GetCANProtocalSetting(std::string path);

  int ParseCanFrame(const int64_t &timestamp, const struct can_frame *frame,
                    CANFrameRaw *raw);

  typedef int (CANParser::*CANFrameCvtFunc)(const CANRawDataMap &raw_data_map,
                                            CANFrameRaw &raw);
  std::vector<CANFrameCvtFunc> can_cvt_func_;

  virtual int ParseWheelAngle(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw);
  virtual int ParseGear(const CANRawDataMap &raw_data_map, CANFrameRaw &raw);
  virtual int ParseVehSpeed(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw);
  virtual int ParseUDLCLeftStatus(const CANRawDataMap &raw_data_map,
                                  CANFrameRaw &raw);
  virtual int ParseUDLCRightStatus(const CANRawDataMap &raw_data_map,
                                   CANFrameRaw &raw);
  virtual int ParseTurnSignal(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw);
  virtual int ParseAcc(const CANRawDataMap &raw_data_map, CANFrameRaw &raw);
  virtual int ParseYawRateValid(const CANRawDataMap &raw_data_map,
                                CANFrameRaw &raw);
  virtual int ParseYawRate(const CANRawDataMap &raw_data_map, CANFrameRaw &raw);
  virtual int ParseYawRateOffset(const CANRawDataMap &raw_data_map,
                                 CANFrameRaw &raw);
  virtual int ParseLateralAcc(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw);
  virtual int ParseBrakePedalPosition(const CANRawDataMap &raw_data_map,
                                      CANFrameRaw &raw);
  virtual int ParseAccPedalPosition(const CANRawDataMap &raw_data_map,
                                    CANFrameRaw &raw);
  virtual int ParsePowerMode(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw);
  virtual int ParseHazardLight(const CANRawDataMap &raw_data_map,
                               CANFrameRaw &raw);
  virtual int ParseHeadWiper(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw);
  virtual int ParseOutsideTemperatrue(const CANRawDataMap &raw_data_map,
                                      CANFrameRaw &raw);
  virtual int ParseLightLevel(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw);
  virtual int ParseHMWEnable(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw);
  virtual int ParseFCWEnable(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw);
  virtual int ParseLDWEnable(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw);
  virtual int ParsePCWEnable(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw);
  virtual int ParseUFCWEnable(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw);
  virtual int ParseWSFrontLeft(const CANRawDataMap &raw_data_map,
                               CANFrameRaw &raw);
  virtual int ParseWSFrontRight(const CANRawDataMap &raw_data_map,
                                CANFrameRaw &raw);
  virtual int ParseWSRearLeft(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw);
  virtual int ParseWSRearRight(const CANRawDataMap &raw_data_map,
                               CANFrameRaw &raw);
  bool ParseRawData(const struct can_frame *frame,
                    const CANSettingMap *setting_map, CANRawDataMap &data_map);
#if 1
  virtual int Parseseconds(const CANRawDataMap &raw_data_map, CANFrameRaw &raw);
  virtual int Parsenanoseconds(const CANRawDataMap &raw_data_map,
                               CANFrameRaw &raw);
#endif

  typedef RawInfo (CANParser::*CANFrameFilterFunc)(int8_t &rst_flg,
                                                   const RawInfo &rawinfo);

  std::vector<CANParser::CANFrameFilterFunc> can_filter_func_;

  virtual RawInfo FilterSpeed(int8_t &rst_flg, const RawInfo &rawinfo);
  virtual RawInfo FilterTurnSignal(int8_t &rst_flg, const RawInfo &rawinfo);

  virtual int ParseYawRateDirection(const CANRawDataMap &raw_data_map,
                                    CANFrameRaw &raw);
  virtual int ParseWheelAngleDirection(const CANRawDataMap &raw_data_map,
                                       CANFrameRaw &raw);
  CANFrameRaw last_can_frame_raw_;
  virtual int ParseWheelPulseFL(const CANRawDataMap &raw_data_map,
                                CANFrameRaw &raw);
  virtual int ParseWheelPulseFR(const CANRawDataMap &raw_data_map,
                                CANFrameRaw &raw);
  virtual int ParseWheelPulseRL(const CANRawDataMap &raw_data_map,
                                CANFrameRaw &raw);
  virtual int ParseWheelPulseRR(const CANRawDataMap &raw_data_map,
                                CANFrameRaw &raw);
  virtual int ParseWheelPulseFLDir(const CANRawDataMap &raw_data_map,
                                   CANFrameRaw &raw);
  virtual int ParseWheelPulseFRDir(const CANRawDataMap &raw_data_map,
                                   CANFrameRaw &raw);
  virtual int ParseWheelPulseRLDir(const CANRawDataMap &raw_data_map,
                                   CANFrameRaw &raw);
  virtual int ParseWheelPulseRRDir(const CANRawDataMap &raw_data_map,
                                   CANFrameRaw &raw);

  virtual int ParseImuAccX(const CANRawDataMap &raw_data_map, CANFrameRaw &raw);
  virtual int ParseImuAccY(const CANRawDataMap &raw_data_map, CANFrameRaw &raw);
  virtual int ParseImuAccZ(const CANRawDataMap &raw_data_map, CANFrameRaw &raw);
  virtual int ParseImuGyroX(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw);
  virtual int ParseImuGyroY(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw);
  virtual int ParseImuGyroZ(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw);
  virtual int ParseCANYaw(const CANRawDataMap &raw_data_map, CANFrameRaw &raw);
  virtual int ParseOdoPosX(const CANRawDataMap &raw_data_map, CANFrameRaw &raw);
  virtual int ParseOdoPosY(const CANRawDataMap &raw_data_map, CANFrameRaw &raw);
  virtual int ParseOdoPosZ(const CANRawDataMap &raw_data_map, CANFrameRaw &raw);
  virtual int ParseOdoSpeed(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw);
  virtual int ParseOdoYawRate(const CANRawDataMap &raw_data_map,
                              CANFrameRaw &raw);
  virtual int ParseLeftTurnSignal(const CANRawDataMap &raw_data_map,
                                  CANFrameRaw &raw);
  virtual int ParseRightTurnSignal(const CANRawDataMap &raw_data_map,
                                   CANFrameRaw &raw);
  virtual int ParseRoiStartX(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw);
  virtual int ParseRoiStartY(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw);
  virtual int ParseRoiWidth(const CANRawDataMap &raw_data_map,
                            CANFrameRaw &raw);
  virtual int ParseRoiHeight(const CANRawDataMap &raw_data_map,
                             CANFrameRaw &raw);
  virtual int ParseRoiRollingCount(const CANRawDataMap &raw_data_map,
                                   CANFrameRaw &raw);
  virtual int ParseLowBeamStatus(const CANRawDataMap &raw_data_map,
                                 CANFrameRaw &raw);
  virtual int ParseHighBeamStatus(const CANRawDataMap &raw_data_map,
                                  CANFrameRaw &raw);
  virtual int ParseAutoHeadlightStatus(const CANRawDataMap &raw_data_map,
                                       CANFrameRaw &raw);
};

class CANSender {
 public:
  CANSender();
  ~CANSender();
  bool Init(const char *device, int can_protocol,
            struct sockaddr_can *sockname);
  void Fini(void);
  void SendData(ADASOutputCAN *data, int can_protocol);

 protected:
  int can_socket_;
  struct sockaddr_can sockname_j1939_;
  struct sockaddr_can peername_j1939_;
};

typedef struct J1939_CANID_TypeTag {
  unsigned int sa_ : 8;
  unsigned int pgn_ : 18;
  unsigned int prio_ : 3;
  unsigned int reserved_ : 3;
} J1939_CANID_Type;
typedef struct J1939_PDU1_TypeTag {
  unsigned int sa_ : 8;
  unsigned int da_ : 8;
  unsigned int pgn_ : 10;
  unsigned int prio_ : 3;
  unsigned int reserved_ : 3;
} J1939_PDU1_Type;
typedef struct J1939_PDU2_TypeTag {
  unsigned int sa_ : 8;
  unsigned int pgn_ : 18;
  unsigned int prio_ : 3;
  unsigned int reserved_ : 3;
} J1939_PDU2_Type;
typedef struct J1939_PDUx_TypeTag {
  unsigned int sa_ : 8;
  unsigned int ps_ge_ : 8;
  unsigned int pf_ : 8;
  unsigned int dp_ : 1;
  unsigned int r_ : 1;
  unsigned int prio_ : 3;
  unsigned int reserved_ : 3;
} J1939_PDUx_Type;

typedef union CAN_ID_TypeTag {
  uint16_t _w[2];
  uint8_t _c[4];
  uint32_t _can_id_;
  J1939_CANID_Type j193_can_id_;
  J1939_PDU1_Type j1939_pdu1_;
  J1939_PDU2_Type j1939_pdu2_;
  J1939_PDUx_Type j1939_pdux_;
} CAN_ID_Type;

enum DebugCANIDType {
  DebugCANID_frame_start = 0,
  DebugCANID_car_header = 1,
  DebugCANID_lane_header = 2,
  DebugCANID_ped_header = 3,
  DebugCANID_frame_end = 4,
  DebugCANID_COUNT = 5
};

enum DirectionType { DirectionType_Positive = 0, DirectionType_Negative = 1 };

}  // end of namespace CAN
#ifndef __QNX__
// typedef TemplateMessage<ADASOutputCAN *> ADASOutputCANMessage;
#endif
// typedef TemplateTimestampMessage<CAN::CANFrame> VelStatusMessage;
// typedef TemplateTimestampMessage<struct can_frame> CANInputMessage;
// typedef TemplateTimestampMessage<struct canfd_frame> CANFDInputMessage;
// typedef TemplateMessage<std::vector<
//  std::shared_ptr<CANInputMessage> > > CANInputVecMessage;
// typedef TemplateMessage<std::vector<
// std::shared_ptr<CANFDInputMessage> > > CANFDInputVecMessage;
}  // end of namespace HobotADAS

#endif  // SRC_COMMON_CAN_H_
