#ifndef B2EB1FA5_BE00_461E_8FF5_DC765FD2D83F
#define B2EB1FA5_BE00_461E_8FF5_DC765FD2D83F

#include "fusion_track.h"

struct SensorFrame;

class FusionTrackManager
{
public:
    static FusionTrackManager& GetInstance()
    {
        static FusionTrackManager mgr;
        return mgr;
    }

    /**
     * @brief 获取已有轨迹跟踪器中，与当前传感器类型相关的跟踪器下标列表
     * @param type 传感器类型
     * @return 跟踪器下标列表
     */
    std::vector<uint32_t> GetFusionTracksOfSensor(const SensorType type);
    /**
     * @brief 根据传感器数据，创建新的轨迹跟踪器
     * @param sensor_list 传感器数据列表
     * @param unassigned_objects_idx  需要创建轨迹的传感器数据下标
     */
    void CreateNewTracks(const SensorFrame& sensor_list, const std::vector<uint32_t>& unassigned_objects_idx);

    /**
     * @brief 移除已失效的轨迹跟踪器
     */
    void RemoveDeadTracks();

    /**
     * @brief 融合指向同一目标轨迹的轨迹跟踪器
     * @param cur_type 当前帧数据传感器类型
     */
    void FuseSameTracks(const SensorType cur_type);

    /**
     * @brief 置0操作，供测试用例使用
     */
    void Clear();

private:
    FusionTrackManager() {}
    ~FusionTrackManager() {}

    /**
     * @brief 根据传感器数据，创建轨迹跟踪器
     * @param sensor_object 传感器数据
     */
    void CreateOneTrack(const SensorObject& sensor_object);

    /**
     * @brief 创建轨迹跟踪器时，分配跟踪器ID
     * @return  分配的跟踪器ID
     */
    uint32_t GetNewTrackId();

    /**
     * @brief 获取该类传感器可融合的跟踪器
     * @param type 传感器类型
     * @param single_track_index 只有该传感器轨迹的跟踪器下标列表
     * @param fused_track_index 缺少该传感器轨迹的跟踪器下标列表
     */
    void GetIntegratableTracksBySensor(const SensorType type, std::vector<uint32_t>& single_track_index, std::vector<uint32_t>& fused_track_index);

    /**
     * @brief 融合相同目标的轨迹的跟踪器
     * @param fused_idx  缺少该传感器轨迹的跟踪器下标
     * @param single_idx 只有该传感器轨迹的跟踪器下标
     * @param type 传感器类型
     */
    void FuseOneSameTrack(const uint32_t fused_idx, const uint32_t single_idx, const SensorType type);

public:
    std::vector<FusionTrack> track_list;
};


#endif /* B2EB1FA5_BE00_461E_8FF5_DC765FD2D83F */
