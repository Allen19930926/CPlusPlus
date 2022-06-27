#ifndef EF48505C_E775_45A5_B76D_0E37D4888491
#define EF48505C_E775_45A5_B76D_0E37D4888491

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

#endif /* EF48505C_E775_45A5_B76D_0E37D4888491 */
