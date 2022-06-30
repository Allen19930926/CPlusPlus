#ifndef EF48505C_E775_45A5_B76D_0E37D4888491
#define EF48505C_E775_45A5_B76D_0E37D4888491

#include <cstring>

namespace CDDFusion
{

#define CAMERA_OBJ_VEHI_NUM     20
#define GSENTRY_OBJ_VEHI_NUM    20


struct EventMessage
{
    EventMessage(const uint32_t id, const char* data_, const uint16_t len):msgid(id), msglen(len)
    {
        // printf("msgid is : %u", msgid);
        if (data != nullptr)
        {
            data = new uint8_t[msglen];
            memcpy(data, data_, msglen);
        }
    }

    ~EventMessage()
    {
        delete [] data;
    }
public:
    uint32_t msgid;
    uint16_t msglen;
    uint8_t* data;
};

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

struct CddFusionRepo
{
    CDDFusionGSentryObj v2xObjVehi[GSENTRY_OBJ_VEHI_NUM];
    CDDFusionCameraObj  j3ObjVehi[CAMERA_OBJ_VEHI_NUM];
};
}

#endif /* EF48505C_E775_45A5_B76D_0E37D4888491 */
