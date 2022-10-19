#include "fusion_system.h"
#include "sensor_adapter.h"
#include "ipc_data.h"
#include <glog/logging.h>

void FusionSystem::Fuse(uint8_t* data, uint16_t len)
{
    if (len != sizeof(CDD_Fusion_ObjInfo_Array40))
    {
        LOG(ERROR) << "datalen mismatch!!";
        return ;
    }

    SensorFrame frame;
    if (SensorAdapter::Transformation(data, frame))
    {
        LOG(ERROR) << "Add frame failed!!";
        return ;
    }

    AddSensorFrame(frame);

    std::vector<SensorFrame> frames;
    GetLatestFrames(frame.time_stamp, frames);  // 后期需要考虑，track和sensor时间间隔过大问题

    for (const auto& frame : frames)
    {
        FuseFrame(frame);
    }

    RemoveLostTracks();
}

void FusionSystem::AddSensorFrame(const SensorFrame& frame)
{
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);
}

void GetLatestFrames(const uint32_t time_stamp, std::vector<SensorFrame>& frames)
{
    SensorDataManager::GetInstance().QueryLatestFrames(time_stamp, frames);
}

void FusionSystem::FuseFrame(const SensorFrame& sensor_list)
{
    std::vector<uint32_t> sensor_track_index_list = FusionTrackManager::GetInstance().
    GetFusionTracksOfSensor(sensor_list.sensor_type);

    predictor.Predict(sensor_list.time_stamp, sensor_track_index_list);

    matcher.Associate(sensor_list, sensor_track_index_list);

    const std::vector<TrackMatchPair>& match_results = matcher.GetMatchResult();
    updater.UpdateAssignedTracks(sensor_list, match_results);

    const std::vector<uint32_t>& unmatched_tracks = matcher.GetUnmatchedTracks();
    updater.UpdateUnassignedTracks(unmatched_tracks);

    const std::vector<uint32_t>& unmatched_objects = matcher.GetUnmatchedObjects();
    CreateNewTracks(sensor_list, unmatched_objects);
}

void FusionSystem::CreateNewTracks(const SensorFrame& sensor_list, const std::vector<uint32_t>& unassigned_objects_idx)
{
    FusionTrackManager::GetInstance().CreateNewTracks(sensor_list, unassigned_objects_idx);
}

void FusionSystem::RemoveLostTracks()
{
    FusionTrackManager::GetInstance().RemoveDeadTracks();
}

// void FusionSystem::FuseTracks()
// {
//     FusionTrackManager::GetInstance().RemoveDeadTracks();
// }
