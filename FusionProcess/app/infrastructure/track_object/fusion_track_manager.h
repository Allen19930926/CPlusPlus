#ifndef B2EB1FA5_BE00_461E_8FF5_DC765FD2D83F
#define B2EB1FA5_BE00_461E_8FF5_DC765FD2D83F

#include "fusion_track.h"
#include <random>
// #include <vector>

class FusionTrackManager
{
public:
    FusionTrackManager& GetInstance()
    {
        static FusionTrackManager mgr;
        return mgr;
    }

    void GetFusionTracksOfSensor(const SensorType type, std::vector<uint32_t>& track_index);
    void CreateNewTracks(std::vector<SensorObject>&  unassigned_objects);
    void RemoveDeadTracks();

public:
    std::vector<FusionTrack> track_list;

private:
    FusionTrackManager();
    ~FusionTrackManager() {}
    void CreateOneTrack(const SensorObject& sensor_object);
    uint32_t GetNewTrackId();

private:
    std::mt19937 track_id_generator;
};


#endif /* B2EB1FA5_BE00_461E_8FF5_DC765FD2D83F */
