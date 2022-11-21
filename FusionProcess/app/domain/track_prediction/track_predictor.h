#ifndef BBEA619D_17BD_4CA4_8874_1DA72C61756C
#define BBEA619D_17BD_4CA4_8874_1DA72C61756C

#include "infrastructure/kalman_filter/kalman_filter.h"

class FusionTrack;
class FusionTrackKfData;

class TrackPredictor
{
public:
    /**
     * @brief 将列表中的融合轨迹同步到当前时间帧，并进行运动补偿
     * @param timestamp 当前时间帧
     * @param type 当前帧传感器类型
     * @param track_index_list 待更新融合轨迹列表
     * @return 时间同步是否执行
     */
    bool Predict(const uint32_t timestamp, SensorType type, const std::vector<uint32_t>& track_index_list);
private:
    /**
     * @brief 单个融合轨迹时间同步
     * @param timestamp 当前时间帧
     * @param yaw_rate 自车角速度
     * @param track 融合轨迹跟踪器
     */
    void PredictOneTrack(const uint32_t timestamp, const float yaw_rate, FusionTrack& track);
    /**
     * @brief 融合轨迹时间同步
     * @param time_diff 时间差
     * @param yaw_rate 自车角速度
     * @param track 融合轨迹跟踪器
     */
    void PredictFusedTraj(const uint32_t time_diff, const float yaw_rate, FusionTrack& track);
    /**
     * @brief 传感器轨迹时间同步
     * @param time_diff 时间差
     * @param yaw_rate 自车角速度
     * @param track 融合轨迹跟踪器
     */
    void PredictSensorTraj(const uint32_t time_diff, const float yaw_rate, FusionTrack& track);
    /**
     * @brief 对预测轨迹进行运动补偿，补偿角速度带来的位置速度偏移
     * @param time_diff 时间差
     * @param yaw_rate 自车角速度
     * @param kf_data 轨迹
     */
    void CompensateEgoMotion(const uint32_t time_diff, const float yaw_rate, FusionTrackKfData& kf_data);
private:
    KalmanFilter kf_filter;
};

#endif /* BBEA619D_17BD_4CA4_8874_1DA72C61756C */
