#ifndef IPC_DATA_H_
#define IPC_DATA_H_

typedef unsigned char boolean;
typedef signed char sint8;
typedef signed short sint16;
typedef signed int sint32;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef signed int sint8_least;
typedef signed int sint16_least;
typedef signed int sint32_least;
typedef unsigned int uint8_least;
typedef unsigned int uint16_least;
typedef unsigned int uint32_least;
typedef float float32;
typedef double float64;
typedef signed long long sint64;
typedef unsigned long long uint64;
typedef uint32 usize;

#define VAR(vartype, memclass) vartype

typedef VAR(uint8, TYPEDEF) UInt8;
typedef VAR(uint32, TYPEDEF) UInt32;
typedef VAR(float32, TYPEDEF) Float32;
typedef VAR(boolean, TYPEDEF) Boolean;

#pragma pack(push, 4)

typedef struct
{
  VAR(UInt8, TYPEDEF) De_stCoreAShutdown_u8;
  VAR(UInt8, TYPEDEF) De_stCoreANormalOperation_u8;
  VAR(UInt32, TYPEDEF) De_reserved_u32;
} CDD_CoreAStatus_BUS;

typedef struct
{
  VAR(Boolean, TYPEDEF) De_ADAS_FCW_u8;
  VAR(UInt8, TYPEDEF) De_gSentry_Warning_u8;
  VAR(UInt8, TYPEDEF) De_FCW_AEB_FuncSts_u8;
  VAR(Boolean, TYPEDEF) De_AEB_Triger_u8;
  VAR(Boolean, TYPEDEF) De_AEB_Response_u8;
  VAR(UInt8, TYPEDEF) De_CACC_CACCStatus_u8;
  VAR(UInt8, TYPEDEF) De_CACC_ACCStatus_u8;
  VAR(UInt8, TYPEDEF) De_CACC_IDAStatus_u8;
  VAR(UInt8, TYPEDEF) De_CACC_Decision_u8;
  VAR(UInt8, TYPEDEF) De_CACC_CACCOn_Rsp_u8;
  VAR(UInt8, TYPEDEF) De_CACC_CACCOff_Rsp_u8;
  VAR(UInt8, TYPEDEF) De_CACC_CACCResume_Rsp_u8;
  VAR(UInt8, TYPEDEF) De_CACC_SpeedSet_Rsp_u8;
  VAR(UInt8, TYPEDEF) De_CACC_TimeGapSet_Rsp_u8;
  VAR(UInt8, TYPEDEF) De_CACC_IDAOn_Rsp_u8;
  VAR(UInt8, TYPEDEF) De_CACC_IDAOff_Rsp_u8;
  VAR(UInt8, TYPEDEF) De_CACC_SportMode_Rsp_u8;
  VAR(UInt32, TYPEDEF) De_ResponseCounter_u32;
} FuncCoord_FAM_HMI_Info;

typedef struct
{
  VAR(UInt32, TYPEDEF) De_Timestamp_u32;
  VAR(UInt8, TYPEDEF) De_ID_u8;
  VAR(Float32, TYPEDEF) De_conf_f32;
  VAR(UInt8, TYPEDEF) De_measurement_status_u8;
  VAR(UInt32, TYPEDEF) De_life_time_u32;
  VAR(UInt8, TYPEDEF) De_source_u8;
  VAR(UInt8, TYPEDEF) De_ObjectType_u8;
  VAR(UInt8, TYPEDEF) De_ObjectMovingStatus_u8;
  VAR(Float32, TYPEDEF) De_length_f32;
  VAR(Float32, TYPEDEF) De_width_f32;
  VAR(Float32, TYPEDEF) De_height_f32;
  VAR(Float32, TYPEDEF) De_Yaw_f32;
  VAR(Float32, TYPEDEF) De_conf_yaw_f32;
  VAR(Float32, TYPEDEF) De_yaw_rate_f32;
  VAR(Float32, TYPEDEF) De_dx_f32;
  VAR(Float32, TYPEDEF) De_dy_f32;
  VAR(Float32, TYPEDEF) De_vx_f32;
  VAR(Float32, TYPEDEF) De_vy_f32;
  VAR(Float32, TYPEDEF) De_ax_f32;
  VAR(Float32, TYPEDEF) De_ay_f32;
  VAR(Float32, TYPEDEF) De_dxVariance_f32;
  VAR(Float32, TYPEDEF) De_dyVariance_f32;
  VAR(Float32, TYPEDEF) De_vxVariance_f32;
  VAR(Float32, TYPEDEF) De_vyVariance_f32;
  VAR(Float32, TYPEDEF) De_axVariance_f32;
  VAR(Float32, TYPEDEF) De_ayVariance_f32;
  VAR(Float32, TYPEDEF) De_curr_lane_f32;
  VAR(Float32, TYPEDEF) De_ttc_f32;
  VAR(Float32, TYPEDEF) De_ettc_f32;
  VAR(Boolean, TYPEDEF) De_CIPV_u8;
} CDD_Fusion_ObjInfo_BUS;

typedef VAR(CDD_Fusion_ObjInfo_BUS, TYPEDEF) CDD_Fusion_ObjInfo_Array40[40];
typedef VAR(CDD_Fusion_ObjInfo_BUS, TYPEDEF) CDD_Fusion_ObjInfo_Array90[90];

typedef struct
{
  VAR(UInt32, TYPEDEF) De_Timestamp_u32;
  VAR(UInt8, TYPEDEF) De_LaneValid_u8;
  VAR(UInt8, TYPEDEF) De_LaneID_u8;
  VAR(UInt32, TYPEDEF) De_Life_time_u32;
  VAR(UInt8, TYPEDEF) De_LaneType_u8;
  VAR(Float32, TYPEDEF) De_Conf_f32;
  VAR(Float32, TYPEDEF) De_LaneWidth_f32;
  VAR(UInt8, TYPEDEF) De_LaneColor_u8;
  VAR(UInt8, TYPEDEF) De_LaneMarking_u8;
  VAR(Float32, TYPEDEF) De_start_Xpt_f32;
  VAR(Float32, TYPEDEF) De_start_Ypt_f32;
  VAR(Float32, TYPEDEF) De_LaneDist_f32;
  VAR(Float32, TYPEDEF) De_C0_f32;
  VAR(Float32, TYPEDEF) De_C1_f32;
  VAR(Float32, TYPEDEF) De_C2_f32;
  VAR(Float32, TYPEDEF) De_C3_f32;
} CDD_Fusion_LaneInfo_BUS;

typedef VAR(CDD_Fusion_LaneInfo_BUS, TYPEDEF) CDD_Fusion_LaneInfo_Array4[4];

typedef struct
{
  VAR(Float32, TYPEDEF) De_DistanceToEndLine_f32;
} CDD_DistanceToEndLine;

typedef struct
{
  VAR(UInt8, TYPEDEF) De_TrafficLightSt_u8;
  VAR(Float32, TYPEDEF) De_RedTime_f32;
  VAR(Float32, TYPEDEF) De_GreenTime_f32;
  VAR(Float32, TYPEDEF) De_YellowTime_f32;
} CDD_CurntLaneTrafficLightInfo_BUS;

typedef struct
{
  VAR(UInt8, TYPEDEF) De_WarningType_u8;
  VAR(UInt8, TYPEDEF) De_level_u8;
  VAR(UInt8, TYPEDEF) De_TargetID_u8;
  VAR(UInt32, TYPEDEF) De_reserved_u32;
} CDD_gSentry_WarningInfo_BUS;

typedef struct
{
  VAR(Boolean, TYPEDEF) De_AEB_SwtRequest_u8;
  VAR(UInt8, TYPEDEF) De_FCW_SnvtySet_u8;
  VAR(Boolean, TYPEDEF) De_CACC_Switch_u8;
  VAR(Boolean, TYPEDEF) De_IDA_Switch_u8;
  VAR(Boolean, TYPEDEF) De_CACC_Resume_u8;
  VAR(Boolean, TYPEDEF) De_CACC_Cancel_u8;
  VAR(UInt8, TYPEDEF) De_TimeGapSet_f32;
  VAR(Float32, TYPEDEF) De_Vset_f32;
  VAR(Float32, TYPEDEF) De_VDis_f32;
  VAR(UInt32, TYPEDEF) De_RequestCounter_u32;
} SignalInput_HMI_BUS;

#define IPC_MAX_SYSTEM_ERROR_NUM 256
typedef struct
{
  VAR(UInt8, TYPEDEF) De_Fim_Fids_u8[IPC_MAX_SYSTEM_ERROR_NUM];
} IPC_System_Error;

typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float pitchAngle;
    float rollAngle;
    float headingAngle;
    float locatHeight;
    uint32 time;
    double latitude;
    double longitude;
    float northSpd;
    float eastSpd;
    float toGroundSpd;
    uint8 gpsFlagPos;
    uint8 numSv;
    uint8 gpsFlagHeading;
    uint8 gpsAge;
    uint8 gpsStatus;
    uint8 status;
    float stdLat;
    float stdLon;
    float stdLocateHeight;
    float stdHeading;
} IPC_GNSS_Data;

typedef struct {
    uint8 utcYear;
    uint8 utcMonth;
    uint8 utcDay;
    uint8 utcHour;
    uint8 utcMin;
    uint8 utcSec;
    uint16 utcMilliSec;
    uint32 reserved;
} IPC_GNSS_UTC;

typedef struct
{
  VAR(Float32, TYPEDEF) De_ego_vxMs_f32;
  VAR(Float32, TYPEDEF) De_ego_axMs2_f32;
  VAR(Float32, TYPEDEF) De_ego_ayMs2_f32;
  VAR(Float32, TYPEDEF) De_Kaptraj_f32;
  VAR(Float32, TYPEDEF) De_YawrateFilter_f32;
  VAR(Float32, TYPEDEF) De_RoadSlope_f32;
} EVH_SubjectInfo_BUS;

typedef struct 
{
    int32_t transmissionState;    ///< 档位信息
                                  ///< netual (0),          --空挡
                                  ///< park (1),            --泊车档
                                  ///< forwardGears (2),    --前进档
                                  ///< reverseGears (3),    --后退档
                                  ///< reserved1 (4),
                                  ///< reserved2 (5),
                                  ///< reserved3 (6),
                                  ///< unavailable (7), --not-equipped or unavailable value,
                                  ///< Any related speed is relative to the vehicle reference fram used

    int32_t steeringWheelAngle;   ///< 方向盘转角信息
                                  ///< Units of 1.5 degrees, a range of -189 to +189 degrees
                                  ///< +001 = +1.5 deg
                                  ///< -126 = -189 deg and beyond
                                  ///< +126 = +189 deg and beyond
                                  ///< +127 to be used for unavailable

    int32_t brakePedalStatus;     ///< 刹车踏板状态
                                  ///< unavailable (0),  --vehicle brake pedal detector is unavailable
                                  ///< off (1),          --vehicle's brake pedal is not pressed
                                  ///< on (2),           --vehicle's brake pedal is pressed

    int32_t brakeAppliedStatus;   ///< 车轮制动情况(bit string)
                                  ///< unavailable (0),  --When set, the brake applied status is unavailable
                                  ///< leftFront (1),    --Left Front Active
                                  ///< leftRear (2),     --Left Rear Active
                                  ///< rightFront (3),   --Right Front Active
                                  ///< rightRear (4),    --Right Rear Active

    int32_t tractionControlStatus;  ///< 牵引力控制系统
                                    ///< unavailable (0),   --Not Equipped with traction control
                                    ///<                    --or traction control status is unavailable
                                    ///< off (1),           --traction control is Off
                                    ///< on (2),            --traction control is On(but not Engaged)
                                    ///< engaged (3),       --traction control is Engaged

    int32_t antiLockBrakeStatus;  ///< 防抱死系统状态
                                  ///< unavailable (0),   --Not Equipped with ABS brakes
                                  ///<                    --or ABS brakes status is unavailable
                                  ///< off (1),           --vehicle's ABS is Off
                                  ///< on (2),            --vehicle's ABS is On(but not Engaged)
                                  ///< engaged (3),       --vehicle's ABS control is Engaged or any wheel

    int32_t stabilityControlStatus;  ///< 车辆稳定性控制系统作用情况
                                     ///< unavailable (0),   --Not Equipped with SC
                                     ///<                    --or SC status is unavailable
                                     ///< off (1),           --vehicle's SC is Off
                                     ///< on (2),            --vehicle's SC is On or active(but not Engaged)
                                     ///< engaged (3),       --vehicle's SC control is Engaged

    int32_t brakeBoostApplied;       ///< 刹车助力系统作用情况
                                     ///< unavailable (0),   --Not Equipped with brake boost
                                     ///<                    --or brake boost data is unavailable
                                     ///< off (1),           --vehicle's brake boost is Off
                                     ///< on (2),            --vehicle's brake boost is On(applied)

    int32_t auxiliaryBrakesStatus;   ///< 刹车辅助系统状态（通常为手刹）
                                     ///< unavailable (0),   --Not Equipped with Aux Brakes
                                     ///<                    --or Aux Brakes status is unavailable
                                     ///< off (1),           --vehicle's Aux Brakes is Off
                                     ///< on (2),            --vehicle's Aux Brakes is On(Engaged)
                                     ///< reserved (3),

    int32_t eventHazardLights;       ///< 车辆警示灯亮起（双闪）事件
                                     ///< unavailable (0),   --Not support, the status data is unavailable
                                     ///< unsatisfied (1),   --the event is unsatisfied
                                     ///< satisfied (2),     --the event is satisfied
                                     ///< reserved (3),

    int32_t eventABSactivated;       ///< 防abs被激活状态(触发并超过100毫秒)事件
                                     ///< unavailable (0),   --Not support, the status data is unavailable
                                     ///< unsatisfied (1),   --the event is unsatisfied
                                     ///< satisfied (2),     --the event is satisfied
                                     ///< reserved (3),

    int32_t eventTractionControlLoss;  ///< 电子系统控制牵引力触发(触发并超过100毫秒)事件
                                       ///< unavailable (0),   --Not support, the status data is unavailable
                                       ///< unsatisfied (1),   --the event is unsatisfied
                                       ///< satisfied (2),     --the event is satisfied
                                       ///< reserved (3),

    int32_t eventStabilityControlactivated;    ///< 车身稳定控制系统被触发(触发并超过100毫秒)事件
                                               ///< unavailable (0),   --Not support, the status data is unavailable
                                               ///< unsatisfied (1),   --the event is unsatisfied
                                               ///< satisfied (2),     --the event is satisfied
                                               ///< reserved (3),

    int32_t eventHardBraking;       ///< 紧急制动(急刹车且减速度大于0.4G)事件
                                    ///< unavailable (0),   --Not support, the status data is unavailable
                                    ///< unsatisfied (1),   --the event is unsatisfied
                                    ///< satisfied (2),     --the event is satisfied
                                    ///< reserved (3),

    int32_t eventFlatTire;          ///< 爆胎(至少1个轮胎爆胎)事件
                                    ///< unavailable (0),   --Not support, the status data is unavailable
                                    ///< unsatisfied (1),   --the event is unsatisfied
                                    ///< satisfied (2),     --the event is satisfied
                                    ///< reserved (3),

    int32_t eventDisabledVehicle;   ///< 车辆故障无法行驶事件
                                    ///< unavailable (0),   --Not support, the status data is unavailable
                                    ///< unsatisfied (1),   --the event is unsatisfied
                                    ///< satisfied (2),     --the event is satisfied
                                    ///< reserved (3),

    int32_t eventAirBagDeployment;  ///< 安全气囊弹开(至少一个安全气囊弹出)事件
                                    ///< unavailable (0),   --Not support, the status data is unavailable
                                    ///< unsatisfied (1),   --the event is unsatisfied
                                    ///< satisfied (2),     --the event is satisfied
                                    ///< reserved (3),

    int32_t eventWipersStatus;      ///< 雨刷开启状态
                                    ///< unavailable (0),   --Not support, the status data is unavailable
                                    ///< off (1),   --wippers are off
                                    ///< on (2),    --wippers are on
                                    ///< reserved (3),

    int32_t exteriorLights;     ///< 车灯状态(bitstring)
                                ///< lowBeamHeadlightsOn (0),
                                ///< highBeamHeadlightsOn (1),
                                ///< leftTurnSignalOn (2),
                                ///< rightTurnSignalOn (3),
                                ///< hazardSignalOn (4),
                                ///< automaticLightControlOn (5),
                                ///< daytimeRunningLightsOn (6),
                                ///< fogLightOn (7),
                                ///< parkingLightsOn (8)

    int32_t vehicleSpeed;      ///< 车速
                               ///来自轮速的车辆速度大小 分辨率0.02m/s，无效值8191 

    int32_t enduranceMileage;    ///< 续航里程 分辨率0.1km
                                 ///< --Units of 0.1km,a range of 0 to 3276.6km
                                 ///< --00001 = 0.1km
                                 ///< --4000 = 400.0km
                                 ///< --32767 to be used for unavailable 

} IPC_CAN_Data;

#pragma pack(pop)

#endif
