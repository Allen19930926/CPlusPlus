#include <glog/logging.h>
#include <Eigen/Dense>
#include "infrastructure/track_object/fusion_track_manager.h"
#include "track_predictor.h"

namespace
{
    const float MILLISECONDS_IN_ONE_SECOND = 1000;
    const float EGO_MOTION_COMPENSE_GATE = 0.1;
    float ego_vehi_yaw_rate = 0;
}

bool TrackPredictor::Predict(const uint32_t timestamp, SensorType type)
{

    std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;

    if (track_list.empty())
    {
        LOG(ERROR) << "Track list is empty, predict quit!!";
        return false;
    }

	for (auto& track : track_list)
	{
        PredictOneTrack(timestamp, ego_vehi_yaw_rate, track);
	}
    return true;
}

void TrackPredictor::PredictOneTrack(const uint32_t timestamp, const float yaw_rate, FusionTrack& track)
{
    const uint32_t time_diff = timestamp - track.time_stamp;

    track.time_stamp = timestamp;
    PredictFusedTraj(time_diff, yaw_rate, track);
    PredictSensorTraj(time_diff, yaw_rate, track);
}

void TrackPredictor::PredictFusedTraj(const uint32_t time_diff, const float yaw_rate, FusionTrack& track)
{
    FusionTrackKfData kf_data;
    kf_data.x_poster << track.position, track.velocity, track.acceleration;
    kf_data.p_poster(0,0)   = track.pos_variance(0);
    kf_data.p_poster(1,1)   = track.pos_variance(1);
    kf_data.p_poster(2,2)   = track.vel_variance(0);
    kf_data.p_poster(3,3)   = track.vel_variance(1);
    kf_data.p_poster(4,4)   = track.acc_variance(0);
    kf_data.p_poster(5,5)   = track.acc_variance(1);

    kf_filter.Predict(time_diff, kf_data);
    CompensateEgoMotion(time_diff, yaw_rate, kf_data);

    track.position      << kf_data.x_prior(0), kf_data.x_prior(1);
    track.velocity      << kf_data.x_prior(2), kf_data.x_prior(3);
    track.acceleration  << kf_data.x_prior(4), kf_data.x_prior(5);
    track.pos_variance(0)   = kf_data.p_prior(0,0);
    track.pos_variance(1)   = kf_data.p_prior(1,1);
    track.vel_variance(0)   = kf_data.p_prior(2,2);
    track.vel_variance(1)   = kf_data.p_prior(3,3);
    track.acc_variance(0)   = kf_data.p_prior(4,4);
    track.acc_variance(1)   = kf_data.p_prior(5,5);
}

void TrackPredictor::PredictSensorTraj(const uint32_t time_diff, const float yaw_rate, FusionTrack& track)
{
    for (auto& trajetory : track.sensor_trajetories)
    {
        if (!trajetory.kf_data.exist)
        {
            continue;
        }
        kf_filter.Predict(time_diff, trajetory.kf_data);
        CompensateEgoMotion(time_diff, yaw_rate, trajetory.kf_data);
    }
}

void TrackPredictor::CompensateEgoMotion(const uint32_t time_diff, const float yaw_rate, FusionTrackKfData& kf_data)
{
    if (yaw_rate < EGO_MOTION_COMPENSE_GATE)
    {
        return;
    }
    const float yaw_diff = time_diff / MILLISECONDS_IN_ONE_SECOND * yaw_rate;
    KfMatrix trans_matrix = KfMatrix::Identity();
    // construct transformation matrix for displacement
    trans_matrix(0, 0) = cos(yaw_diff);
    trans_matrix(0, 1) = -sin(yaw_diff);
    trans_matrix(1, 0) = sin(yaw_diff);
    trans_matrix(1, 1) = cos(yaw_diff);
    // construct transformation matrix for velocity
    trans_matrix(2, 0) = 0;
    trans_matrix(2, 1) = -yaw_rate;
    trans_matrix(3, 0) = yaw_rate;
    trans_matrix(3, 1) = 0;
    trans_matrix(2, 2) = cos(yaw_diff);
    trans_matrix(2, 3) = -sin(yaw_diff);
    trans_matrix(3, 2) = sin(yaw_diff);
    trans_matrix(3, 3) = cos(yaw_diff);
    // construct transformation matrix for acceleration
    trans_matrix(4, 4) = cos(yaw_diff);
    trans_matrix(4, 5) = -sin(yaw_diff);
    trans_matrix(5, 4) = sin(yaw_diff);
    trans_matrix(5, 5) = cos(yaw_diff);

    // LOG(INFO) << "\n" << trans_matrix;

    // LOG(INFO) << "\n" << kf_data.x_prior;

    kf_data.x_prior = trans_matrix * kf_data.x_prior;
}

