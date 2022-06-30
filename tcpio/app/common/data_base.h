#ifndef C90FDE72_D43C_4497_97F1_AAB3FFE76AA2
#define C90FDE72_D43C_4497_97F1_AAB3FFE76AA2

#include "v2x_data_struct.h"
#include "cdd_fusion.h"
#include "can_info_struct.h"
#include "hb_data.h"

// DataRepo设计为只有事件IO访问，因为不做互斥
class DataRepo
{
public:
    static DataRepo& GetInstance()
    {
        static DataRepo instance;
        return instance;
    }
    V2X::V2xData& GetV2xData() {return v2xData;}
    CDDFusion::CddFusionRepo& GetCddFusionData() {return cddFusionData;}
    CAN::HostVehiclePos& GetHostVehicle() {return host;}

private:
    DataRepo() {}

private:
    CAN::HostVehiclePos host;
    V2X::V2xData v2xData;
    gohigh::Obstacles cameraObstacles;
    CDDFusion::CddFusionRepo cddFusionData;
};

#endif /* C90FDE72_D43C_4497_97F1_AAB3FFE76AA2 */
