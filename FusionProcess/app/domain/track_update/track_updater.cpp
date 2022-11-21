#include <glog/logging.h>

#include "infrastructure/sensor_object/sensor_object.h"
#include "infrastructure/track_object/fusion_track.h"
#include "infrastructure/track_object/fusion_track_manager.h"

#include "track_updater.h"

void TrackUpdater::UpdateAssignedTracks(const SensorFrame& sensor_list, const std::vector<TrackMatchPair>& match_results)
{
    std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    uint32_t result_num = static_cast<uint32_t>(match_results.size());
    uint32_t track_num = static_cast<uint32_t>(track_list.size());
    uint32_t object_num = static_cast<uint32_t>(sensor_list.sensors.size());

    if (result_num == 0)
    {
        LOG(ERROR) << "There is no tracks to update!!";
        return;
    }

    if (result_num > track_num || result_num > object_num)
    {
        LOG(ERROR) << "matched results' num: " << result_num << " is greater than track num: "
            << track_num << " or objects' num: " << object_num;
        return;
    }

    for (const auto pair : match_results)
    {
        UpdateTrackWithMeas(sensor_list.sensors[pair.object_idx], track_list[pair.track_idx]);
    }

}

void TrackUpdater::UpdateUnassignedTracks(const SensorType type, const std::vector<uint32_t>& unmatched_tracks)
{
    std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    uint32_t unmatched_track_num = static_cast<uint32_t>(unmatched_tracks.size());
    uint32_t track_num = static_cast<uint32_t>(track_list.size());

    if (unmatched_track_num == 0)
    {
        LOG(ERROR) << "There is no tracks to update!!";
        return;
    }

    if (unmatched_track_num > track_num)
    {
        LOG(ERROR) << "unmatched results' num" << unmatched_track_num
            << " is greater than track num: " << track_num;
        return;
    }

    for (const auto track_idx : unmatched_tracks)
    {
        UpdateTrackWithoutMeas(type, track_list[track_idx]);
    }
}

void TrackUpdater::UpdateTrackWithMeas(const SensorObject& meas, FusionTrack& track)
{
    if (meas.sensor_type >= static_cast<uint32_t>(SensorType::MAX))
    {
        LOG(ERROR) << "\n sensor type is error, sensor object info:"
                   << "\n id = " << static_cast<uint32_t>(meas.id) << ", sensor_type = " << meas.sensor_type
                   << "\n x = " <<meas.position(0) << ", y = " << meas.position(1);
        return ;
    }
    UpdateTrackCommonAttr(meas, track);
    UpdateTrackTraj(meas, track);
}

void TrackUpdater::UpdateTrackCommonAttr(const SensorObject& meas, FusionTrack& track)
{
    const SensorType type = static_cast<SensorType>(meas.sensor_type);
    track.yaw               = meas.yaw;
    track.yaw_rate          = meas.yaw_rate;
    track.is_dead           = 0;
    track.size              = meas.size;
    track.fusion_status    |= 1 << meas.sensor_type;
    track.track_duration++;
    if (type == SensorType::CAMERA)
    {
        track.cipv = meas.cipv;
        track.object_type = meas.object_type;
    }
    if (type == SensorType::V2X)
    {
        track.fusion_status |= meas.measurement_status;
        track.object_type   = meas.object_type;
    }
}

void TrackUpdater::UpdateTrackTraj(const SensorObject& meas, FusionTrack& track)
{
    // update each sensor trajectory
    for (uint32_t i=0; i<track.sensor_trajetories.size(); i++)
    {
        SensorTrajetory& cur_sensor_trajectory = track.sensor_trajetories[i];
        if (!cur_sensor_trajectory.kf_data.exist)
        {
            continue;
        }
        if (i == meas.sensor_type)
        {
            cur_sensor_trajectory.coasting_age = 0;
            cur_sensor_trajectory.sensor_id    = meas.id;
            kf_filter.Update(meas, cur_sensor_trajectory.kf_data);
        }
        else
        {
            kf_filter.Update(cur_sensor_trajectory.kf_data);
        }
    }

    // fuse track trajetory and current sensor trajetory
    SensorTrajetory& cur_sensor_trajectory = track.sensor_trajetories[meas.sensor_type];

    track.position(0)       = cur_sensor_trajectory.kf_data.x_poster(0);
    track.position(1)       = cur_sensor_trajectory.kf_data.x_poster(1);
    track.velocity(0)       = cur_sensor_trajectory.kf_data.x_poster(2);
    track.velocity(1)       = cur_sensor_trajectory.kf_data.x_poster(3);
    track.acceleration(0)   = cur_sensor_trajectory.kf_data.x_poster(4);
    track.acceleration(1)   = cur_sensor_trajectory.kf_data.x_poster(5);
    track.pos_variance(0)   = cur_sensor_trajectory.kf_data.p_poster(0,0);
    track.pos_variance(1)   = cur_sensor_trajectory.kf_data.p_poster(1,1);
    track.vel_variance(0)   = cur_sensor_trajectory.kf_data.p_poster(2,2);
    track.vel_variance(1)   = cur_sensor_trajectory.kf_data.p_poster(3,3);
    track.acc_variance(0)   = cur_sensor_trajectory.kf_data.p_poster(4,4);
    track.acc_variance(1)   = cur_sensor_trajectory.kf_data.p_poster(5,5);
}

void TrackUpdater::UpdateTrackWithoutMeas(const SensorType type, FusionTrack& track)
{
    // 标记当前传感器轨迹为滑行状态，如果滑行时间超过3帧，则该传感器轨迹失效，融合轨迹的融合状态退化。
    // 如果融合状态为0，则该融合轨迹消亡，不再进行更新。
    uint32_t type_seq = static_cast<uint32_t>(type);

    SensorTrajetory& cur_sensor_trajectory = track.sensor_trajetories[type_seq];
    cur_sensor_trajectory.coasting_age++;

    if (cur_sensor_trajectory.coasting_age > 2)
    {
        track.fusion_status ^= 1 << type_seq;
        cur_sensor_trajectory.kf_data.exist = false;
    }

    if (track.fusion_status == 0)
    {
        track.is_dead = 1;
        track.track_duration = 0;
        return ;
    }

    for (uint32_t i=0; i<track.sensor_trajetories.size(); i++)
    {
        if (!track.sensor_trajetories[i].kf_data.exist)
        {
            continue;
        }
        kf_filter.Update(track.sensor_trajetories[i].kf_data);
    }
}
