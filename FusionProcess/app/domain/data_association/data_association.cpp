#include <unordered_map>
#include <glog/logging.h>

#include "infrastructure/track_object/fusion_track.h"
#include "infrastructure/mahalanobis/mahalanobis.h"
#include "infrastructure/track_object/fusion_track_manager.h"

#include "data_association.h"

namespace
{
    double mahal_dist_threshold = 10;        // 匹配结果阈值
    double edcl_dist_threshold = 20;    // 欧式距离阈值
}

void DataAssociation::Associate(const SensorFrame& frame, const std::vector<uint32_t>& sensor_track_index_list)
{
    match_result.clear();
    unmatched_tracks_index.clear();
    unmatched_objects_index.clear();
    uint32_t track_num = static_cast<uint32_t>(sensor_track_index_list.size());
    uint32_t object_num = static_cast<uint32_t>(frame.sensors.size());
    if (track_num == 0)
    {
        LOG(ERROR) << "There is no track to associate!!";
        for (uint32_t i=0; i<frame.sensors.size(); i++)
        {
            unmatched_objects_index.push_back(i);
        }
        return;
    }
    if (object_num == 0)
    {
        LOG(ERROR) << "There is no object to associate!!";
        for (uint32_t i=0; i<sensor_track_index_list.size(); i++)
        {
            unmatched_tracks_index.push_back(sensor_track_index_list[i]);
        }
        return;
    }

    DoIdAssign(frame, sensor_track_index_list);
    // 对未匹配上的 ID 做匈牙利匹配
    DoKahnMunkresMatch(frame);
}

const std::vector<TrackMatchPair>& DataAssociation::GetMatchResult()
{
    return match_result;
}

const std::vector<uint32_t>& DataAssociation::GetUnmatchedTracks()
{
    return unmatched_tracks_index;
}

const std::vector<uint32_t>& DataAssociation::GetUnmatchedObjects()
{
    return unmatched_objects_index;
}

void DataAssociation::DoIdAssign(const SensorFrame& frame, const std::vector<uint32_t>& sensor_track_index_list)
{
    using SensorId = uint32_t;
    using TrackIndex = uint32_t;
    const uint8_t type = static_cast<uint8_t>(frame.sensor_type);
    std::unordered_map<SensorId, std::vector<TrackIndex>> first_assign_result;
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;

    std::vector<bool> sensor_flag(frame.sensors.size(), false);
    std::vector<bool> track_flag(sensor_track_index_list.size(), false);


    for (uint32_t i=0; i<frame.sensors.size(); i++)
    {
        for (uint32_t j=0; j<sensor_track_index_list.size(); j++)
        {
            if (frame.sensors[i].id == track_list[sensor_track_index_list[j]].sensor_trajetories[type].sensor_id)
            {
                sensor_flag[i] = true;
                track_flag[j]  = true;
                first_assign_result[i].push_back(sensor_track_index_list[j]);
            }
        }
    }

    for (uint32_t i=0; i<sensor_flag.size(); i++)
    {
        if (!sensor_flag[i])
        {
            unmatched_objects_index.push_back(i);
        }
    }

    for (uint32_t i=0; i<track_flag.size(); i++)
    {
        if (!track_flag[i])
        {
            unmatched_tracks_index.push_back(sensor_track_index_list[i]);
        }
    }

    for (const auto& pair : first_assign_result)
    {
        if (pair.second.size() > 1)
        {
            unmatched_objects_index.push_back(pair.first);
            std::for_each(pair.second.begin(), pair.second.end(), [this](const uint32_t idx)
            {
                unmatched_tracks_index.push_back(idx);
            });
            continue;
        }

        if (CheckMahalDist(frame.sensors[pair.first], track_list[pair.second[0]]))
        {
            unmatched_tracks_index.push_back(pair.second[0]);
            unmatched_objects_index.push_back(pair.first);
            continue;
        }
        match_result.push_back({pair.first, pair.second[0]});
    }

}

bool DataAssociation::CheckMahalDist(const SensorObject& xSensorObject, const FusionTrack& xFusionTrack)
{
    KfVector xPoint1, xPoint2;

    // 组装马氏距离的参数
    xPoint1 << xSensorObject.position, xSensorObject.velocity, xSensorObject.acceleration;
    xPoint2 << xFusionTrack.position, xFusionTrack.velocity, xFusionTrack.acceleration;

    KfMatrix xCovaiance;
    xCovaiance.setZero();
    xCovaiance(0, 0) = xFusionTrack.pos_variance(0, 0);
    xCovaiance(1, 1) = xFusionTrack.pos_variance(1, 1);
    xCovaiance(2, 2) = xFusionTrack.vel_variance(0, 0);
    xCovaiance(3, 3) = xFusionTrack.vel_variance(1, 1);
    xCovaiance(4, 4) = xFusionTrack.acc_variance(0, 0);
    xCovaiance(5, 5) = xFusionTrack.acc_variance(1, 1);

    return Mahalanobis::GetMahalDistance(xPoint1, xPoint2, xCovaiance) > mahal_dist_threshold;
}

void DataAssociation::DoKahnMunkresMatch(const SensorFrame& frame)
{
    if (unmatched_objects_index.size() == 0 || unmatched_tracks_index.size() == 0)
    {
        LOG(ERROR) << "No unmatched objects/tracks";
        return ;
    }

    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    std::size_t order = std::max(unmatched_objects_index.size(), unmatched_tracks_index.size());
    Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Constant(order, order, 0xFFFFFFFF);

    for (uint32_t i=0; i<unmatched_objects_index.size(); i++)
    {
        const SensorObject& object = frame.sensors[i];
        KfVector object_state_vec;
        object_state_vec << object.position, object.velocity, object.acceleration;
        for (uint32_t j=0; j<unmatched_tracks_index.size(); j++)
        {
            const FusionTrack& track = track_list[unmatched_tracks_index[j]];
            KfVector track_state_vec;
            track_state_vec << track.position, track.velocity, track.acceleration;
            double edcl_dist = Mahalanobis::GetEdclideanDistance(object_state_vec, track_state_vec);
            if (edcl_dist >= edcl_dist_threshold)
            {
                continue;
            }
            KfVector co_variance_vec;
            co_variance_vec <<  track.pos_variance(0,0),  track.pos_variance(1,1),
                                track.vel_variance(0,0),  track.vel_variance(1,1),
                                track.acc_variance(0,0),  track.vel_variance(1,1);
            KfMatrix co_variance = co_variance_vec.asDiagonal();
            double mahal_dist = Mahalanobis::GetMahalDistance(object_state_vec, track_state_vec, co_variance);
            if (mahal_dist >= mahal_dist_threshold)
            {
                continue;
            }
            cost_matrix(i, j) = mahal_dist;
        }
    }

    LOG(INFO) << "\n" << cost_matrix(0,0);

    Hungarian hungarian(cost_matrix);
    auto pair_result = hungarian.Solve();
    std::vector<uint32_t> hungarian_res(&pair_result.second[0], pair_result.second.data() + pair_result.second.cols() * pair_result.second.rows());

    std::vector<uint32_t> unmatched_tracks_index_cc(unmatched_tracks_index);
    std::vector<uint32_t> unmatched_objects_index_cc(unmatched_objects_index);
    for (uint32_t i=0; i< hungarian_res.size(); i++)
    {
        if (cost_matrix(i, hungarian_res[i]) <= mahal_dist_threshold)
        {
            unmatched_objects_index_cc.erase(unmatched_objects_index_cc.begin() + i);
            unmatched_tracks_index_cc.erase(unmatched_tracks_index_cc.begin() + hungarian_res[i]);
            match_result.push_back({i, unmatched_tracks_index[hungarian_res[i]]});
        }
    }
    unmatched_objects_index.swap(unmatched_objects_index_cc);
    unmatched_tracks_index.swap(unmatched_tracks_index_cc);

}
