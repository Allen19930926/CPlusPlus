#include "event_dispatcher.h"
#include <cstdio>
#include "data_base.h"
#include "v2x_fusion_algorithm.h"
#include "camera_fusion_algorithm.h"
#include "v2x_adas_event_macro.h"
#include "event_queue.h"


void EventDispatcher::ProcessMessage()
{
    CDDFusion::EventMessage&& msg = CDD_FUSION_EVENT_QUEUE.wait_and_get_front();
    printf ("msgid = %u\n", msg.msgid);
    switch(msg.msgid)
    {
        // for v2x
        case EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT:     ProcessGSentrySatatus(msg.data, msg.msglen); break;
        case EV_GSENTRY_ADAS_CALC_MAPINFO_REPORT:       ProcessHostVehiExtraMapInfo(msg.data, msg.msglen); break;
        case EV_GSENTRY_ADAS_SPATINFO_REPORT:           ProcessSpatInfo(msg.data, msg.msglen); break;
        case EV_GSENTRY_ADAS_OBJECT_VEHICLE_REPORT:     ProcessObjVehiInfo(msg.data, msg.msglen); break;
        case EV_GSENTRY_ADAS_EGOVEHI_MAPINFO_REPORT:    ProcessHostVehiMapInfo(msg.data, msg.msglen); break;
        case EV_GSENTRY_ADAS_OBJVEHI_MAPINFO_REPORT:    ProcessObjVehiMapInfo(msg.data, msg.msglen); break;
        case EV_GSENTRY_ADAS_WARNING_REPORT:            ProcessGSentryWarningInfo(msg.data, msg.msglen); break;

        // for camera
        case 23434:                                     ProcessJ3CameraData(msg.data, msg.msglen); break;

        // for CAN
        case 45657:                                     ProcessCanHostVehicleInfo(msg.data, msg.msglen); break;
        default: break;
    }
}

void EventDispatcher::ProcessGSentrySatatus(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || len != sizeof(V2X::GSentryStatus))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.status, buf, len);
    printf("recieve gSentry Status msg!\n");
}

void EventDispatcher::ProcessHostVehiExtraMapInfo(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || len != sizeof(V2X::MapAddResult))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.mapAddRes, buf, len);
    printf("recieve gSentry Calc Map msg!\n");
}

void EventDispatcher::ProcessSpatInfo(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || (len != sizeof(V2X::AdasSpatInfo) * ADAS_SPAT_INFO_NUM))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.spatInfo[0], buf, len);
    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    // 融合数据只用了当前车道下一个红绿灯信息，因此只取v2x数据的第一个红绿灯信息
    V2xFusionAlgo::TransV2xSpat2CddSpat(v2xData.spatInfo[0], fusion.spatInfo);
    printf("recieve gSentry Spat msg!\n");
}

void EventDispatcher::ProcessObjVehiInfo(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || (len != sizeof(V2X::ObjVehMapInfo) * ADAS_OBJ_VEH_INFO_NUM))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.objVehicle[0], buf, len);

    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    CAN::HostVehiclePos& host = DataRepo::GetInstance().GetHostVehicle();

    uint16_t i = 0;
    v2xData.objVehiNum = 0;
    while(v2xData.objVehicle[i].localId != 0)
    {
        V2xFusionAlgo::TransV2xVehi2CddVehi(v2xData.objVehicle[i], host, fusion.v2xObjVehi[i]);
        printf("recieve gSentry ObjVehiInfo msg!\n");
        v2xData.objVehiNum++;
        i++;
    }
}

void EventDispatcher::ProcessHostVehiMapInfo(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || (len != sizeof(V2X::EgoVehMapInfo)))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.egoMap, buf, len);
}

void EventDispatcher::ProcessObjVehiMapInfo(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || len != sizeof(V2X::ObjVehMapInfo))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.objMap, buf, len);
}

void EventDispatcher::ProcessGSentryWarningInfo(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || len != sizeof(V2X::WarningInfo))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    memcpy(&v2xData.warningInfo, buf, len);
    V2xFusionAlgo::TransV2xWarn2CddWarn(v2xData.warningInfo, fusion.gSentryWarningInfo);
}

void EventDispatcher::ProcessJ3CameraData(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || len != sizeof(gohigh::Obstacles))
    {
        return ;
    }
    gohigh::Obstacles& obstacles = DataRepo::GetInstance().GetCameraObstacles();
    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    memcpy(&obstacles, buf, len);

    // 使用小顶堆，遍历所有目标障碍物，按和本车距离进行排序，堆顶为最近障碍物
    auto compare = [](gohigh::Obstacle left, gohigh::Obstacle right)
    {
        float lx = left.world_info.position.x;
        float ly = left.world_info.position.y;
        float rx = right.world_info.position.x;
        float ry = right.world_info.position.y;
        return (lx * lx + ly * ly) > (rx * rx + ry * ry);
    };
    std::priority_queue<gohigh::Obstacle, std::vector<gohigh::Obstacle>, decltype(compare)> nearestVehis(compare);
    for (uint16_t i=0; i<obstacles.obstacle_num; i++)
    {
        nearestVehis.push(obstacles.obstacles[i]);
    }

    // 从小顶堆堆顶抛出20个障碍物，填充到fusion结构体
    for (uint16_t i=0; i<CAMERA_OBJ_VEHI_NUM && !nearestVehis.empty(); i++)
    {
        const auto& objectVehicle = nearestVehis.top();
        CameraFusionAlgo::TransCamera2CddObstacle(objectVehicle, obstacles.timestamp, fusion.j3ObjVehi[i]);
        nearestVehis.pop();
    }
}

void EventDispatcher::ProcessCanHostVehicleInfo(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || len != sizeof(CAN::HostVehiclePos))
    {
        return ;
    }
    CAN::HostVehiclePos& hostPos = DataRepo::GetInstance().GetHostVehicle();
    memcpy(&hostPos, buf, len);
}

