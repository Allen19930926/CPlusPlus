#include "v2x_data_dispatcher.h"
#include <cstdio>
#include <queue>
#include <mutex>
#include <condition_variable>
#include "data_base.h"
#include "v2x_fusion_algorithm.h"

extern std::queue<CDDFusion::EventMessage> eventQueue;
extern std::mutex mtx;

void V2xDataDispatcher::ProcessV2xMessage()
{
    CDDFusion::EventMessage& msg = eventQueue.front();
    switch(msg.msgid)
    {
        case 0x101: ProcessGSentrySatatus(msg.data, msg.msglen); break;
        default: break;
    }
    eventQueue.pop();
}

void V2xDataDispatcher::ProcessGSentrySatatus(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || len != sizeof(V2X::GSentryStatus))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.status, buf, len);
    printf("recieve gSentry Status msg!\n");
}

void V2xDataDispatcher::ProcessCalcMapResult(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || len != sizeof(V2X::MapAddResult))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.mapAddRes, buf, len);
    printf("recieve gSentry Calc Map msg!\n");
}

void V2xDataDispatcher::ProcessSpatInfo(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || (len != sizeof(V2X::AdasSpatInfo) * ADAS_SPAT_INFO_NUM))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.spatInfo[0], buf, len);
    printf("recieve gSentry Spat msg!\n");
}

void V2xDataDispatcher::ProcessObjVehiInfo(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || (len != sizeof(V2X::ObjVehMapInfo) * ADAS_OBJ_VEH_INFO_NUM))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.objVehicle[0], buf, len);

    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    CAN::HostVehiclePos& host = DataRepo::GetInstance().GetHostVehicle();

    for (uint16_t i=0; i<ADAS_OBJ_VEH_INFO_NUM; i++)
    {
        if (v2xData.objVehicle[i].localId == 0)
        {
            printf("valid vehicles is done!\n");
            break;
        }
        V2xFusionAlgo::TransV2xVehi2CddVehi(v2xData.objVehicle[i], host, fusion.v2xObjVehi[i]);
    }
    printf("recieve gSentry ObjVehiInfo msg!\n");
}
