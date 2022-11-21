#ifndef B0907941_05B5_45D5_8DF9_5908E756D9A2
#define B0907941_05B5_45D5_8DF9_5908E756D9A2

#include "infrastructure/track_object/fusion_track_manager.h"

struct TrackManagerMocker
{
public:
    TrackManagerMocker():track_manager(FusionTrackManager::GetInstance()) {}
    void MockCreateTracks(const uint32_t num, const SensorType type);
    void MockRemoveTracks();
    void ClearOperation();
private:
    FusionTrackManager& track_manager;
};

#endif /* B0907941_05B5_45D5_8DF9_5908E756D9A2 */
