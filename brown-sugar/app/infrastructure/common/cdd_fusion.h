#ifndef EF48505C_E775_45A5_B76D_0E37D4888491
#define EF48505C_E775_45A5_B76D_0E37D4888491
#include "ipc_data.h"

enum ObjectSource
{
    CAMERA_SOURCE = 0,
    OBU_V2X_SOURCE = 1,
    RSM_V2X_SOURCE = 2,
    INVALID_SOURCE = 3
};

enum ObjectType
{
    ObstacleType_VehicleRear = 0,
    ObstacleType_VehicleFull = 1,
    ObstacleType_Pedestrian = 2,
    ObstacleType_Cyclist = 3
};

enum ObjectMovingStatus
{
    MS_INVALID = 0,
    MS_UNKNOWN = 1,
    MS_MOVING = 2,
    MS_STATIONARY = 3,
    MS_STOPPED = 4,
    MS_MOVING_SLOWLY = 5,
    MS_ENUM_SIZE = 6
};

namespace CDDFusion
{

#define ADAS_CAMERA_OBJ_VEHI_NUM     20
#define ADAS_GSENTRY_OBJ_VEHI_NUM    20
#define CDD_LANE_INFO_NUM       4

struct CDDCurntLaneTrafficLightInfo
{
    uint8_t     trafficLightSt;
    float       redTime;
    float       greenTime;
    float       yellowTime;
};

struct CDDFusionLaneInfo
{
    uint32_t timestamp;
    uint8_t laneValid;
    uint8_t laneID;
    uint32_t life_time;
    uint8_t laneType;
    float conf;
    float laneWidth;
    uint8_t laneColor;
    uint8_t laneMarking;
    float start_Xpt;
    float start_Ypt;
    float laneDist;
    float c0;
    float c1;
    float c2;
    float c3;
};

struct CDDgSentryWarningInfo
{
    uint8_t warningType;
    uint8_t level;
    uint8_t targetID;
    uint32_t De_reserved_u32;
};

struct CDDDisToEndLane
{
    float disToEndLane;
};

struct CddFusionRepo
{
    CDD_Fusion_ObjInfo_BUS  cddObjects[ADAS_GSENTRY_OBJ_VEHI_NUM + ADAS_CAMERA_OBJ_VEHI_NUM];
    CDDCurntLaneTrafficLightInfo    spatInfo;
    CDD_Fusion_LaneInfo_Array4 laneInfo;
    CDDgSentryWarningInfo gSentryWarningInfo;
    CDDDisToEndLane   disToEndLane;
};
}

#endif /* EF48505C_E775_45A5_B76D_0E37D4888491 */
