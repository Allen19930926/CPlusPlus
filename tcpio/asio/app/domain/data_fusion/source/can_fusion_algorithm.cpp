#include "can_fusion_algorithm.h"
#include <cstring>
#include "ipc_data.h"

void CanFusionAlgo::ProcessCanHostVehicleInfo(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || len != sizeof(CAN::HostVehiclePos))
    {
        return ;
    }
    CAN::HostVehiclePos& hostPos = DataRepo::GetInstance().GetHostVehicle();
    memcpy(&hostPos, buf, len);
}

void CanFusionAlgo::ProcessRecieveData(MsgType msgType, uint8_t* data, uint16_t len)
{
    if (data == nullptr)
    {
        return ;
    }
    CAN::HostVehiclePos& hostPos = DataRepo::GetInstance().GetHostVehicle();
    hostPos.isHostPosValid = true;

    switch (msgType)
    {
        case MsgType::IPC_GNSS_DATA:
        {
            const IPC_GNSS_Data& gnssData = *reinterpret_cast<IPC_GNSS_Data*>(data);
            hostPos.objectHeadingAngle = gnssData.headingAngle;
            hostPos.longitude = gnssData.longitude;
            hostPos.latitude = gnssData.latitude;
            hostPos.elevation = gnssData.locatHeight;
        }break;
        default:    
            break;
    }
}