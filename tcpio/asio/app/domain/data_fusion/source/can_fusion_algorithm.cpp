#include "can_fusion_algorithm.h"
#include <cstring>

void CanFusionAlgo::ProcessCanHostVehicleInfo(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || len != sizeof(CAN::HostVehiclePos))
    {
        return ;
    }
    CAN::HostVehiclePos& hostPos = DataRepo::GetInstance().GetHostVehicle();
    memcpy(&hostPos, buf, len);
}