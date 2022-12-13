#ifndef C90FDE72_D43C_4497_97F1_AAB3FFE76AA2
#define C90FDE72_D43C_4497_97F1_AAB3FFE76AA2

#include "v2x_data_struct.h"
#include "cdd_fusion.h"
#include "can_info_struct.h"
#include "hb_data.h"
#include <cstring>

struct CalibrateStruct
{
    float dx_gnss_to_rear_center;
    float dy_gnss_to_rear_center;
};

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
    void Clear()
    {
        memset(&host, 0, sizeof(host));
        memset(&v2xData, 0, sizeof(v2xData));
        memset(&cameraObstacles, 0, sizeof(cameraObstacles));
        memset(&cddFusionData, 0, sizeof(cddFusionData));
    }
    CDDFusion::CddFusionRepo& GetCddFusionData() {return cddFusionData;}
    CAN::HostVehiclePos& GetHostVehicle() {return host;}
    gohigh::Obstacles& GetCameraObstacles() {return cameraObstacles;}
    CalibrateStruct& GetCalibrateVariables() {return calibrateVariables;}

private:
    DataRepo() {}
    ~DataRepo() {}
    DataRepo(const DataRepo& ref) = delete;
    DataRepo& operator=(const DataRepo& ref) = delete;

private:
    CAN::HostVehiclePos host;
    V2X::V2xData v2xData;
    gohigh::Obstacles cameraObstacles;
    CDDFusion::CddFusionRepo cddFusionData;
    CalibrateStruct   calibrateVariables;
};

#endif /* C90FDE72_D43C_4497_97F1_AAB3FFE76AA2 */
