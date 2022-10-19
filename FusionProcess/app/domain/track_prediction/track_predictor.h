#ifndef BBEA619D_17BD_4CA4_8874_1DA72C61756C
#define BBEA619D_17BD_4CA4_8874_1DA72C61756C

#include "kalman_filter.h"

class FusionTrack;

class TrackPredictor
{
public:
    void Predict(const uint32_t timestamp, std::vector<uint32_t>& sensor_track_index_list);
private:
    void CompensateEgoMotion(const uint32_t timediff, FusionTrack& track);
private:
    KalmanFilter kf_filter;
};

#endif /* BBEA619D_17BD_4CA4_8874_1DA72C61756C */
