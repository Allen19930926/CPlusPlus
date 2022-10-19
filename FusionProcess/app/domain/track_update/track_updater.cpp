#include "track_updater.h"
#include "sensor_object.h"
#include "fusion_track.h"
#include "fusion_track_manager.h"
#include <glog/logging.h>

void TrackUpdater::UpdateAssignedTracks(const SensorFrame& sensor_list, const std::vector<TrackMatchPair>& match_results)
{
    std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    uint32_t result_num = static_cast<uint32_t>(match_results.size());
    uint32_t track_num = static_cast<uint32_t>(track_list.size());
    uint32_t object_num = static_cast<uint32_t>(sensor_list.sensors.size());

    if (result_num == 0)
    {
        LOG(ERROR) << "There is no tracks to update!!";
        return ;
    }

    if (result_num > track_num || result_num > object_num)
    {
        LOG(ERROR) << "matched results' num" << result_num << " is greater than track num: "
                   << track_num << "or objects' num: " << object_num;
        return ;
    }

}

void TrackUpdater::UpdateUnassignedTracks(const std::vector<uint32_t>& unmatched_tracks)
{
    std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    uint32_t unmatched_track_num = static_cast<uint32_t>(unmatched_tracks.size());
    uint32_t track_num = static_cast<uint32_t>(track_list.size());

    if (unmatched_track_num == 0)
    {
        LOG(ERROR) << "There is no tracks to update!!";
        return ;
    }

    if (unmatched_track_num > track_num)
    {
        LOG(ERROR) << "unmatchde results' num" << unmatched_track_num
                   << " is greater than track num: " << track_num;
        return ;
    }
}
