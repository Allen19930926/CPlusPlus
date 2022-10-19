#ifndef EA965FBF_C562_4F27_A2F8_24205F073955
#define EA965FBF_C562_4F27_A2F8_24205F073955

#include "kuhn_munkres_algo.h"
#include <cstdint>
#include <vector>

class SensorFrame;
class TrackMatchPair;

class DataAssociation
{
public:
    void Associate(const SensorFrame& frame, const vector<uint32_t>& sensor_track_index_list);
    const std::vector<TrackMatchPair>& GetMatchResult();
    const std::vector<uint32_t> & GetUnmatchedTracks();
    const std::vector<uint32_t> & GetUnmatchedObjects();
private:
    KMAlgorithm                     km_matcher;
    std::vector<TrackMatchPair>     match_result;
    std::vector<uint32_t>           unmatched_tracks_index;
    std::vector<uint32_t>           unmatched_objects_index;
};

#endif /* EA965FBF_C562_4F27_A2F8_24205F073955 */
