#include "fusion_track_manager.h"
#include <glog/logging.h>
#include <queue>
#include "infrastructure/mahalanobis/mahalanobis.h"
#include "infrastructure/common/fusion_tools.h"

namespace
{
    const float SAME_TRACK_JUDGE_GATE = 10.0;
    std::deque<uint32_t> track_id_pool {0,1,2,3,4,5,6,7,8,9,
                                        10,11,12,13,14,15,16,17,18,19,
                                        20,21,22,23,24,25,26,27,28,29,
                                        30,31,32,33,34,35,36,37,38,39,
                                        40,41,42,43,44,45,46,47,48,49};
}

std::vector<uint32_t> FusionTrackManager::GetFusionTracksOfSensor(const SensorType type)
{
    std::vector<uint32_t>  sensor_index;
    uint8_t bit_flag = static_cast<uint8_t>(type);
    uint32_t track_num = static_cast<uint32_t>(track_list.size());

    for (uint32_t i = 0; i < track_num; i++)
    {
        if (track_list[i].fusion_status & (1 << bit_flag))
        {
            sensor_index.emplace_back(i);
        }
    }
    return sensor_index;
}

void FusionTrackManager::CreateNewTracks(const SensorFrame& sensor_list, const std::vector<uint32_t>& unassigned_objects_idx)
{
    for (const auto index : unassigned_objects_idx)
    {
        CreateOneTrack(sensor_list.sensors[index]);
    }
}

void FusionTrackManager::RemoveDeadTracks()
{
    auto rm_cond = [](const FusionTrack& track)
                    { 
                        if (track.is_dead != 0)
                        {
                            track_id_pool.push_back(track.track_id);
                            return true;
                        }
                        return false;
                    };
    track_list.erase(std::remove_if(track_list.begin(), track_list.end(), rm_cond), track_list.end());
}

void FusionTrackManager::CreateOneTrack(const SensorObject& sensor_object)
{
    if (track_id_pool.empty())
    {
        return;
    }
    FusionTrack track;
    track.track_id = GetNewTrackId();
    track.time_stamp = sensor_object.time_stamp;
    track.fusion_status = 1 << sensor_object.sensor_type;
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
    SensorTrajetory& track_sensor_info = track.sensor_trajetories[sensor_object.sensor_type];
    track_sensor_info.sensor_id = sensor_object.id;
    track_sensor_info.coasting_age = 0;
    track_sensor_info.kf_data.exist = true;
    track_sensor_info.kf_data.x_poster << sensor_object.position(0), sensor_object.position(1),
                                          sensor_object.velocity(0), sensor_object.velocity(1),
                                          sensor_object.acceleration(0), sensor_object.acceleration(1);
    track_sensor_info.kf_data.p_poster = KfMatrix::Identity();
    track_sensor_info.kf_data.p_poster(0,0) = sensor_object.pos_variance(0);
    track_sensor_info.kf_data.p_poster(1,1) = sensor_object.pos_variance(1);
    track_sensor_info.kf_data.p_poster(2,2) = sensor_object.vel_variance(0);
    track_sensor_info.kf_data.p_poster(3,3) = sensor_object.vel_variance(1);
    track_sensor_info.kf_data.p_poster(4,4) = sensor_object.acc_variance(0);
    track_sensor_info.kf_data.p_poster(5,5) = sensor_object.acc_variance(1);

    track_list.emplace_back(std::move(track));
}

uint32_t FusionTrackManager::GetNewTrackId()
{
    uint32_t id = track_id_pool.front();
    track_id_pool.pop_front();
    return id;
}

void FusionTrackManager::FuseSameTracks(const SensorType cur_type)
{
    std::vector<uint32_t> single_object_track_index;
    std::vector<uint32_t> lack_object_track_index;
    GetIntegratableTracksBySensor(cur_type, single_object_track_index, lack_object_track_index);
    if (lack_object_track_index.empty())
    {
        LOG(INFO) << "there is no tracks need to fuse this sensor data!";
        return;
    }

    Eigen::MatrixXd mal_dist = DistCalcInterface::GetTrackTrackMahalDistance(track_list, lack_object_track_index, single_object_track_index);

    Eigen::MatrixXf::Index row;
    Eigen::MatrixXf::Index col;
    std::vector<uint32_t> erase_id;
    while ( mal_dist.size() > 0)
    {
        if (mal_dist.minCoeff(&row, &col) >= SAME_TRACK_JUDGE_GATE)
        {
            break;
        }
        FuseOneSameTrack(lack_object_track_index[static_cast<uint32_t>(row)],
            single_object_track_index[static_cast<uint32_t>(col)], cur_type);

        erase_id.emplace_back(track_list[single_object_track_index[static_cast<uint32_t>(col)]].track_id);

        FusionTool::RemoveSpecRowAndCol(mal_dist, static_cast<uint32_t>(row), static_cast<uint32_t>(col));
        lack_object_track_index.erase(lack_object_track_index.begin() + row);
        single_object_track_index.erase(single_object_track_index.begin() + col);
    }

    auto rm_cond = [&erase_id](const FusionTrack& track)
    {
        return std::find(erase_id.begin(), erase_id.end(), track.track_id) !=  erase_id.end(); 
    };
    track_list.erase(std::remove_if(track_list.begin(), track_list.end(), rm_cond), track_list.end());
}

void FusionTrackManager::GetIntegratableTracksBySensor(const SensorType type, std::vector<uint32_t>& single_track_index, std::vector<uint32_t>& fused_track_index)
{
    single_track_index.clear();
    fused_track_index.clear();
    uint8_t bit_flag = static_cast<uint8_t>(type);

    for (uint32_t i = 0; i < track_list.size(); i++)
    {
        if (track_list[i].fusion_status & (1 << bit_flag))
        {
            single_track_index.emplace_back(i);
        }
        else
        {
            fused_track_index.emplace_back(i);
        }
    }
}

void FusionTrackManager::FuseOneSameTrack(const uint32_t fused_idx, const uint32_t single_idx, const SensorType type)
{
    const uint32_t track_num = track_list.size();
    if (fused_idx >=  track_num || single_idx>= track_num)
    {
        LOG(ERROR) << "Unexpected index, track num: " << track_num <<" fused_idx: " << fused_idx
                   << ", single_idx: " << single_idx;
        return ;
    }

    FusionTrack& fused_track = track_list[fused_idx];
    FusionTrack& single_track = track_list[single_idx];

    const std::size_t sensor_type = static_cast<std::size_t>(type);
    SensorTrajetory& fused_sensor_info = fused_track.sensor_trajetories[sensor_type];
    const SensorTrajetory& single_sensor_info = single_track.sensor_trajetories[sensor_type];

    fused_track.fusion_status |= single_track.fusion_status;
    fused_sensor_info.sensor_id = single_sensor_info.sensor_id;
    fused_sensor_info.coasting_age = 0;
    fused_sensor_info.kf_data.exist = single_sensor_info.kf_data.exist;
    fused_sensor_info.kf_data.p_poster = single_sensor_info.kf_data.p_poster;
    fused_sensor_info.kf_data.p_prior = single_sensor_info.kf_data.p_prior;
    fused_sensor_info.kf_data.x_poster = single_sensor_info.kf_data.x_poster;
    fused_sensor_info.kf_data.x_prior = single_sensor_info.kf_data.x_prior;
    if (type == SensorType::CAMERA)
    {
        fused_track.cipv = single_track.cipv;
    }
    if (type == SensorType::CAMERA || type == SensorType::V2X)
    {
        fused_track.object_type = single_track.object_type;
    }
}


void FusionTrackManager::Clear()
{
    track_list.clear();

    std::deque<uint32_t> track_ids {0,1,2,3,4,5,6,7,8,9,
                                    10,11,12,13,14,15,16,17,18,19,
                                    20,21,22,23,24,25,26,27,28,29,
                                    30,31,32,33,34,35,36,37,38,39,
                                    40,41,42,43,44,45,46,47,48,49};

    track_id_pool.swap(track_ids);

}

