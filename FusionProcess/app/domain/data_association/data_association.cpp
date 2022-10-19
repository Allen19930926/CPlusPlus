#include "data_association.h"
#include "fusion_track.h"
#include "glog/logging.h"

void DataAssociation::Associate(const SensorFrame& frame, const vector<uint32_t>& sensor_track_index_list)
{
    uint32_t track_num = static_cast<uint32_t>(sensor_track_index_list.size());
    uint32_t object_num = static_cast<uint32_t>(frame.sensors.size());
    if (track_num == 0 || object_num == 0)
    {
        LOG(ERROR) << "There is no track/object to associate!!";
        return ;
    }
}


