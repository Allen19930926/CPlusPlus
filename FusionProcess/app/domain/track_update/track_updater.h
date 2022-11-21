#ifndef DEB39EEC_88E1_4F00_9868_BA1A4BF1AE88
#define DEB39EEC_88E1_4F00_9868_BA1A4BF1AE88

#include <vector>

#include "infrastructure/kalman_filter/kalman_filter.h"

class SensorFrame;
class TrackMatchPair;

class TrackUpdater
{
public:
    /**
     * @brief 根据数据关联结果，对每一个融合轨迹和传感器数据点进行更新和融合
     * @param sensor_list   当前帧传感器数据列表
     * @param match_results 融合轨迹和传感器数据点匹配结果
     */
    void UpdateAssignedTracks(const SensorFrame& sensor_list, const std::vector<TrackMatchPair>& match_results);

    /**
     * @brief 对未匹配成功给的融合轨迹进行无测量更新
     * @param type 当前帧传感器类型
     * @param unmatched_tracks 数据关联未匹配成功的融合轨迹列表
     */
    void UpdateUnassignedTracks(const SensorType type, const std::vector<uint32_t>& unmatched_tracks);
private:

    /**
     * @brief 更新与测量数据匹配的融合轨迹
     * @param meas  当前帧传感器测量
     * @param track 融合轨迹跟踪器
     */
    void UpdateTrackWithMeas(const SensorObject& meas, FusionTrack& track); 
    /**
     * @brief 更新未与测量点匹配的融合轨迹
     * 
     * 更新步骤
     * 1. 更新当前帧传感器轨迹，如果该传感器轨迹已经滑行3帧，则将该传感器轨迹置为无效
     * 2. 遍历所有有效传感器轨迹进行无测量更新，将先验状态*_prior赋值给后验状态*_poster
     * 3. 对融合轨迹进行无测量更新，由于融合轨迹目前设置为一组数据，因此无赋值操作
     * 4. 如果该track没有有效的传感器轨迹，则将track置为无效
     * @param type  当前帧传感器类型
     * @param track 融合轨迹跟踪器
     */
    void UpdateTrackWithoutMeas(const SensorType type, FusionTrack& track);

    /**
     * @brief 更新融合轨迹跟踪器的属性
     * 
     *  特殊属性更新策略：当前帧传感器为camera时，更新cipv标识和object_type,当前帧传感器为v2x时，更新object_type
     * @param meas  当前帧传感器测量
     * @param track 融合轨迹跟踪器
     */
    void UpdateTrackCommonAttr(const SensorObject& meas, FusionTrack& track);

    /**
     * @brief 更新融合轨迹跟踪器的轨迹状态
     * 更新步骤：
     * 1. 对  当前帧传感器轨迹进行有测量更新
     * 2. 对非当前帧传感器轨迹进行无测量更新
     * 3. 使用更新后的当前帧传感器轨迹，更新融合轨迹.
     *    当传感器轨迹和融合轨迹距离低于门限时，替换当前传感器高分辨率的数据；
     *    当传感器轨迹和融合轨迹距离高于门限时，使用简单融合算法融合两条轨迹
     * @param meas  当前帧传感器测量
     * @param track 融合轨迹跟踪器
     */
    void UpdateTrackTraj(const SensorObject& meas, FusionTrack& track);
private:
    KalmanFilter kf_filter;
};

#endif /* DEB39EEC_88E1_4F00_9868_BA1A4BF1AE88 */
