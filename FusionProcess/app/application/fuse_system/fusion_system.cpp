#include <glog/logging.h>

#include "infrastructure/common/ipc_data.h"
#include "infrastructure/sensor_object/sensor_adapter.h"

#include "fusion_system.h"
#include <chrono>


void FusionSystem::Fuse()
{
    uint64_t time_stamp = GetCurrentTime();

    std::vector<SensorFrame> frames;
    GetLatestFrames(time_stamp, frames);  // 后期需要考虑，track和sensor时间间隔过大问题

    for (const auto& frame : frames)
    {
        FuseFrame(frame);
    }
}

void FusionSystem::AddSensorFrame(const SensorFrame& frame)
{
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);
}

void FusionSystem::GetLatestFrames(const uint32_t time_stamp, std::vector<SensorFrame>& frames)
{
    SensorDataManager::GetInstance().QueryLatestFrames(time_stamp, frames);
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

uint64_t FusionSystem::GetCurrentTime()
{
    // 获取操作系统当前时间点（精确到微秒）
    std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds> tpMicro
        = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
    // (微秒精度的)时间点 => (微秒精度的)时间戳
    time_t totalMicroSeconds = tpMicro.time_since_epoch().count();

    uint64_t currentTime = ((uint64_t)totalMicroSeconds)/1000;

    return currentTime;
}