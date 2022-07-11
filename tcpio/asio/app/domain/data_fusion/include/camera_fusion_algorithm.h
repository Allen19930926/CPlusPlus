#ifndef CA074DF7_E7A6_4146_8CCB_A494B3308E05
#define CA074DF7_E7A6_4146_8CCB_A494B3308E05

#include "data_base.h"

class CameraFusionAlgo
{
public:
    static void TransCamera2CddObstacle(const gohigh::Obstacle& camera, const uint32_t timeStamp, CDDFusion::CDDFusionCameraObj& cdd);
};

#endif /* CA074DF7_E7A6_4146_8CCB_A494B3308E05 */
