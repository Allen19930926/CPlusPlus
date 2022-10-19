#include "track_predictor.h"
#include "fusion_track_manager.h"
#include <glog/logging.h>

namespace
{
    const uint32_t ONE_THOUSAND_MILLISECONDS = 1000;
}

void TrackPredictor::Predict(const uint32_t timestamp, std::vector<uint32_t>& sensor_track_index_list)
{
    if (sensor_track_index_list.size() == 0)
    {
        LOG(INFO) << "There is no tracks to predict!!";
        return ;
    }

    std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    const uint32_t time_diff = (timestamp - track_list[sensor_track_index_list[0]].time_stamp)
                                                                    * ONE_THOUSAND_MILLISECONDS;

    for(const auto track_idx : sensor_track_index_list)
    {
        kf_filter.Predict(time_diff, track_list[track_idx]);
        CompensateEgoMotion(time_diff, track_list[track_idx]);
    }
}

void TrackPredictor::CompensateEgoMotion(const uint32_t timediff, FusionTrack& track)
{

}

