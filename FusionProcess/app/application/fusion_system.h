#ifndef C1E75B9A_A0C0_4323_B95D_5C0A6FD95686
#define C1E75B9A_A0C0_4323_B95D_5C0A6FD95686
#include "fusion_track_manager.h"
#include "sensor_data_manager.h"
#include "fusion_track_manager.h"
#include "data_association.h"
#include "track_predictor.h"
#include "track_updater.h"

class FusionSystem
{
public:
    void Fuse(uint8_t* data, uint16_t len);
private:
    void AddSensorFrame(const SensorFrame& frame);
    void GetLatestFrames(const uint32_t time_stamp, std::vector<SensorFrame>& frames);
    void FuseFrame(const SensorFrame& frame);
    void CreateNewTracks(const SensorFrame& sensor_list, const std::vector<uint32_t>& unassigned_objects_idx);
    void RemoveLostTracks();

private:
    TrackPredictor  predictor;
    DataAssociation matcher;
    TrackUpdater    updater;
};

#endif /* C1E75B9A_A0C0_4323_B95D_5C0A6FD95686 */
