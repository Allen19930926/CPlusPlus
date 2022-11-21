#include "track_manager_mock.h"

void TrackManagerMocker::MockCreateTracks(const uint32_t num, const SensorType type)
{
    SensorFrame frame;
    SensorObject obj;
    std::vector<uint32_t> unassigned_idx;

    frame.time_stamp = 0;
    frame.sensor_type = type;
    obj.sensor_type = static_cast<uint32_t>(type);
    for (uint32_t i = 0; i < num; i++)
    {
        obj.id = i;
        unassigned_idx.push_back(i);
        frame.sensors.push_back(obj);
    }

    track_manager.CreateNewTracks(frame, unassigned_idx);
}

void TrackManagerMocker::MockRemoveTracks()
{
    track_manager.RemoveDeadTracks();
}

void TrackManagerMocker::ClearOperation()
{
    track_manager.Clear();
}
