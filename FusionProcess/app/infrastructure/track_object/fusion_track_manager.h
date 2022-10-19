#ifndef B2EB1FA5_BE00_461E_8FF5_DC765FD2D83F
#define B2EB1FA5_BE00_461E_8FF5_DC765FD2D83F

#include "fusion_track.h"
#include <random>

struct SensorFrame;

class FusionTrackManager
{
public:
    static FusionTrackManager& GetInstance()
    {
        static FusionTrackManager mgr;
        return mgr;
    }

    std::vector<uint32_t> GetFusionTracksOfSensor(const SensorType type);
    void GetSingleSensorTracks(const SensorType type, std::vector<uint32_t>& single_track_index, std::vector<uint32_t>& fused_track_index);
    void CreateNewTracks(const SensorFrame& sensor_list, const std::vector<uint32_t>& unassigned_objects_idx);
    void RemoveDeadTracks();

private:
    FusionTrackManager();
    ~FusionTrackManager() {}
    void CreateOneTrack(const SensorObject& sensor_object);
    uint32_t GetNewTrackId();

public:
    std::vector<FusionTrack> track_list;

private:
    std::mt19937 track_id_generator;
};


#endif /* B2EB1FA5_BE00_461E_8FF5_DC765FD2D83F */
