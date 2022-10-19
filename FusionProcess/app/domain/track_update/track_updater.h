#ifndef DEB39EEC_88E1_4F00_9868_BA1A4BF1AE88
#define DEB39EEC_88E1_4F00_9868_BA1A4BF1AE88

#include "kalman_filter.h"
#include <vector>

class SensorFrame;
class TrackMatchPair;

class TrackUpdater
{
public:
    void UpdateAssignedTracks(const SensorFrame& sensor_list, const std::vector<TrackMatchPair>& match_results);
    void UpdateUnassignedTracks(const std::vector<uint32_t>& unmatched_tracks);
private:

private:
    KalmanFilter kf_filter;
};

#endif /* DEB39EEC_88E1_4F00_9868_BA1A4BF1AE88 */
