#include "can_fusion_algorithm.h"
#include <cstring>
#include "ipc_data.h"
#include <iomanip>
#include <glog/logging.h>

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
            if (gnssData.headingAngle < 0) 
            {
                hostPos.objectHeadingAngle = gnssData.headingAngle + 360;
            }
            else 
            {
                hostPos.objectHeadingAngle = gnssData.headingAngle;
            }
            hostPos.longitude = gnssData.longitude;
            hostPos.latitude = gnssData.latitude;
            hostPos.elevation = gnssData.locatHeight;
            LOG(INFO) << " Host GNSS data, objectHeadingAngle: " << std::fixed << std::setprecision(7) << gnssData.headingAngle
                      << ", longitude: " << std::fixed << std::setprecision(7) << gnssData.longitude
                      << ", latitude: " << std::fixed << std::setprecision(7) << gnssData.latitude
                      << ", elevation: " << std::fixed << std::setprecision(7) << gnssData.locatHeight;
        }break;

        case MsgType::IPC_EVH:
        {
            EVH_SubjectInfo_BUS* hostInfo = (EVH_SubjectInfo_BUS*)(data);
            hostPos.speed = hostInfo->De_ego_vxMs_f32;
            hostPos.acc_x = hostInfo->De_ego_axMs2_f32;
            hostPos.acc_y = hostInfo->De_ego_ayMs2_f32;
        }break;
        default: break;
    }
}