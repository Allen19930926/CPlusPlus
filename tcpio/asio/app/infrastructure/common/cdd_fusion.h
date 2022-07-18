#ifndef EF48505C_E775_45A5_B76D_0E37D4888491
#define EF48505C_E775_45A5_B76D_0E37D4888491


namespace CDDFusion
{

#define CAMERA_OBJ_VEHI_NUM     20
#define GSENTRY_OBJ_VEHI_NUM    20
#define CDD_LANE_INFO_NUM       4

struct CDDFusionGSentryObj
{
    uint32_t timestamp;
    uint8_t id;
    float conf;
    uint8_t measurementStatus;
    uint32_t lifeTime;
    float length;
    float width;
    float height;
    float yaw;
    float yawConf;
    float yawRate;
    float dx;
    float dy;
    float vx;
    float vy;
    float ax;
    float ay;
    float dxVariance;
    float dyVariance;
    float vxVariance;
    float vyVariance;
    float axVariance;
    float ayVariance;
    float currLane;
    float ttc;
    float ettc;
    uint8_t cipv;
};

struct CDDFusionCameraObj
{
    uint32_t timestamp;
    uint8_t id;
    float conf;
    uint8_t measurementStatus;
    uint32_t lifeTime;
    float length;
    float width;
    float height;
    float yaw;
    float yawConf;
    float yawRate;
    float dx;
    float dy;
    float vx;
    float vy;
    float ax;
    float ay;
    float dxVariance;
    float dyVariance;
    float vxVariance;
    float vyVariance;
    float axVariance;
    float ayVariance;
    float currLane;
    float ttc;
    float ettc;
    uint8_t cipv;
};

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
};

struct CDDDisToEndLane
{
    float disToEndLane;
};

struct CddFusionRepo
{
    CDDFusionGSentryObj v2xObjVehi[GSENTRY_OBJ_VEHI_NUM];
    uint32_t rsv0;
    CDDFusionCameraObj  j3ObjVehi[CAMERA_OBJ_VEHI_NUM];
    uint32_t rsv1;
    CDDCurntLaneTrafficLightInfo    spatInfo;
    uint32_t rsv2;
    CDDFusionLaneInfo laneInfo[CDD_LANE_INFO_NUM];
    uint32_t rsv3;
    CDDgSentryWarningInfo gSentryWarningInfo;
    uint32_t rsv4;
    CDDDisToEndLane   disToEndLane;
};
}

#endif /* EF48505C_E775_45A5_B76D_0E37D4888491 */
