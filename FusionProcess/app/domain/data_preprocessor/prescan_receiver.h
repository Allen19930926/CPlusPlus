#ifndef PRESCAN_RECEIVER_H
#define PRESCAN_RECEIVER_H

#include <stdint.h>

#define CAMERA_OBJECT_BYTE  (uint32_t)(4 * 7)
#define RADAR_OBJECT_BYTE  (uint32_t)(4 * 7)

#pragma pack(push, 1)
struct PrescanHeader
{
    //uint32_t sensor_type;
    uint32_t TimeStamp;
    uint32_t ObjectNum;
    //uint32_t bodyLength;
};

struct RadarBody
{
    float       Range;
    float       DopplerVelocityY;
    uint32_t    TargetID;
    float       DopplerVelocityX;
    float       TIS_RangeY;
    float       TIS_RangeX;
};

struct CameraBody
{
    float       Range;
    float       RangeY;
    float       RangeX;
    uint32_t    Object_ID;
    float       DopplerVelocityX;
    float       DopplerVelocityY;
};

#define VAR(vartype, memclass) vartype
typedef VAR(CameraBody, TYPEDEF) Camera_ObjInfo_Array[7];
typedef VAR(RadarBody, TYPEDEF) Radar_ObjInfo_Array[7];

struct PrescanRadarFrame
{
    //uint64_t    timeStamp;
    //uint32_t    radar_num;
    PrescanHeader           RadarHeader;
    Radar_ObjInfo_Array     RadarObjects;
};

struct PrescanCameraFrame
{
    //uint64_t    timeStamp;
    //uint32_t    camera_num;
    PrescanHeader           CameraHeader;
    Camera_ObjInfo_Array    CameraObjects;
};

#pragma pack(pop)

class PrescanReceiver {
public:
    void PrescanReadRadarMsg(char* data);
    void PrescanReadCameraMsg(char* data);

    PrescanReceiver() = default;
};

#endif /* PRESCAN_RECEIVER_H */