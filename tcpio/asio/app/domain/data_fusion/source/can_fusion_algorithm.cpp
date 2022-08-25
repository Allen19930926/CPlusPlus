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
        case MsgType::IPC_GNSS_HEADING_PITCH_ROLL:
        {
            const IPC_GNSS_HeadingPitchRoll& hostOrient = *reinterpret_cast<IPC_GNSS_HeadingPitchRoll*>(data);
            hostPos.objectHeadingAngle = hostOrient.headingAngle;
        }break;
        case MsgType::IPC_GNSS_LAT_LONG:
        {
            const IPC_GNSS_LatitudeLongitude& hostLatLon = *reinterpret_cast<IPC_GNSS_LatitudeLongitude*>(data);
            hostPos.longitude = hostLatLon.longitude;
            hostPos.latitude = hostLatLon.latitude;
        }break;
        case MsgType::IPC_GNSS_HEIGHT_TIME:
        {
            const IPC_GNSS_HeightAndTime& hostHeight = *reinterpret_cast<IPC_GNSS_HeightAndTime*>(data);
            hostPos.elevation = hostHeight.locatHeight;
        }break;
        default:    break;
    }
}