#include <glog/logging.h>

#include "infrastructure/common/ipc_data.h"
#include "infrastructure/common/fusion_tools.h"

#include "fusion_system.h"


void FusionSystem::Fuse()
{
    uint64_t time_stamp = FusionTool::GetCurrentTime();

    std::vector<SensorFrame> frames;
    GetLatestFrames(frames);  // 后期需要考虑，track和sensor时间间隔过大问题

    for (const auto& frame : frames)
    {
        FuseFrame(frame);
    }
}

void FusionSystem::GetLatestFrames(std::vector<SensorFrame>& frames)
{
    SensorDataManager::GetInstance().QueryLatestFrames(frames);
}

void FusionSystem::FuseFrame(const SensorFrame& sensor_list)
{
    std::vector<uint32_t> track_index_list = FusionTrackManager::GetInstance().
        GetFusionTracksOfSensor(sensor_list.sensor_type);

    predictor.Predict(sensor_list.time_stamp, sensor_list.sensor_type);

    matcher.Associate(sensor_list, track_index_list);

    const std::vector<TrackMatchPair>& match_results = matcher.GetMatchResult();
    updater.UpdateAssignedTracks(sensor_list, match_results);

    const std::vector<uint32_t>& unmatched_tracks = matcher.GetUnmatchedTracks();
    updater.UpdateUnassignedTracks(sensor_list.sensor_type, unmatched_tracks);
    RemoveLostTracks();

    const std::vector<uint32_t>& unmatched_objects = matcher.GetUnmatchedObjects();
    CreateNewTracks(sensor_list, unmatched_objects);
    FuseSameTracks(sensor_list.sensor_type);
}

void FusionSystem::CreateNewTracks(const SensorFrame& sensor_list, const std::vector<uint32_t>& unassigned_objects_idx)
{
    FusionTrackManager::GetInstance().CreateNewTracks(sensor_list, unassigned_objects_idx);
}

void FusionSystem::RemoveLostTracks()
{
    FusionTrackManager::GetInstance().RemoveDeadTracks();
}

void FusionSystem::FuseSameTracks(const SensorType type)
{
    FusionTrackManager::GetInstance().FuseSameTracks(type);
}

