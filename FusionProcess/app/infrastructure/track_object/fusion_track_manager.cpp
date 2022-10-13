#include "fusion_track_manager.h"
#include <chrono>
#include <glog/logging.h>

FusionTrackManager::FusionTrackManager() : track_id_generator(std::chrono::steady_clock::now().time_since_epoch().count())
{
}

void FusionTrackManager::GetFusionTracksOfSensor(const SensorType type, std::vector<uint32_t>& track_index)
{
    track_index.clear();
    uint8_t bit_flag = static_cast<uint8_t>(type);

    for (uint32_t i=0; i<track_list.size(); i++)
    {
        if (track_list[i].fusion_status & (1 << bit_flag))
        {
            track_index.emplace_back(i);
        }
    }
}

void FusionTrackManager::CreateNewTracks(std::vector<SensorObject>&  unassigned_objects)
{
    for (const auto& sensor_object : unassigned_objects)
    {
        CreateOneTrack(sensor_object);
    }
}

void FusionTrackManager::RemoveDeadTracks()
{
    auto condition = [](const FusionTrack& track) { return track.is_dead != 0; };
    track_list.erase(std::remove_if(track_list.begin(), track_list.end(), condition), track_list.end());
}

void FusionTrackManager::CreateOneTrack(const SensorObject& sensor_object)
{
    if (track_list.size() > 50)
    {
        return ;
    }
    FusionTrack track;
    track.track_id = GetNewTrackId();
    track.time_stamp = sensor_object.time_stamp;
    track.fusion_status = 1 << sensor_object.sensor_type;
    track.coasting_age = 0;
    track.is_dead = 0;
    track.track_duration = 0;
    track.object_type = sensor_object.object_type;
    track.cipv = sensor_object.cipv;
    track.yaw = sensor_object.yaw;
    track.yaw_rate = sensor_object.yaw_rate;
    track.size = sensor_object.size;
    track.position = sensor_object.position;
    track.pos_variance = sensor_object.pos_variance;
    track.velocity = sensor_object.velocity;
    track.vel_variance = sensor_object.vel_variance;
    track.acceleration = sensor_object.acceleration;
    track.acc_variance = sensor_object.acc_variance;

    // todo  init kf data

    track_list.emplace_back(track);
}

uint32_t FusionTrackManager::GetNewTrackId()
{
    uint32_t id = 0;
    for (uint32_t i=0; i<track_list.size(); i++)
    {
        id = track_id_generator();
        auto it = std::find_if(track_list.begin(), track_list.end(), [id](const FusionTrack& track) {return track.track_id == id;});
        if (it == track_list.end())
        {
            break;
        }
    }
    return id;
}

