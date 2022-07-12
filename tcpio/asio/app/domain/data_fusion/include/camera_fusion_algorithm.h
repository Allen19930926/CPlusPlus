#ifndef CA074DF7_E7A6_4146_8CCB_A494B3308E05
#define CA074DF7_E7A6_4146_8CCB_A494B3308E05

#include "data_base.h"

class CameraFusionAlgo
{
public:
    static void ProcessRecieveData(uint8_t* data, uint16_t len) { ProcessJ3CameraData(data, len); }
private:
    static void ProcessJ3CameraData(uint8_t* buf, uint16_t len);
    static void TransCamera2CddObstacle(const gohigh::Obstacle& camera, const uint32_t timeStamp, CDDFusion::CDDFusionCameraObj& cdd);
};

#endif /* CA074DF7_E7A6_4146_8CCB_A494B3308E05 */
