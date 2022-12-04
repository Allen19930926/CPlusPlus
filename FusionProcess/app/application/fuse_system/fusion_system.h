#ifndef C1E75B9A_A0C0_4323_B95D_5C0A6FD95686
#define C1E75B9A_A0C0_4323_B95D_5C0A6FD95686

#include "infrastructure/sensor_object/sensor_data_manager.h"
#include "infrastructure/track_object/fusion_track_manager.h"
#include "domain/data_association/data_association.h"
#include "domain/track_prediction/track_predictor.h"
#include "domain/track_update/track_updater.h"

class FusionSystem
{
public:
    /**
     * @brief 融合主程序
     * 
     *1. 接收外部传递的传感器信息，转换成本程序的SensorObjectFrame数据
     *2. 将接收的当前帧所有的传感器数据存储到SensorManager管理，并获取所有传感器类型的最近一帧数据，按照时间和类型排序
     *3. 逐帧融合传感器数据
     *4. 发布可信的轨迹点状态信息
     */
    void Fuse();
private:
    /**
     * @brief 添加当前帧所有的传感器数据到SensorManager，进行存储和管理
     * @param frame 当前帧所有的传感器数据
     */

    void AddSensorFrame(const SensorFrame& frame);
    /**
     * @brief 获取所有传感器类型的最近一帧数据
     * @param time_stamp 截止时间戳
     * @param frames 获取的结果
     */
    void GetLatestFrames(const uint32_t time_stamp, std::vector<SensorFrame>& frames);

    /**
     * @brief 融合一帧传感器数据
     * @param frame 当前帧所有的传感器数据
     */
    void FuseFrame(const SensorFrame& frame);

    /**
     * @brief 根据传感器数据，创建新的轨迹跟踪器
     * @param sensor_list 传感器数据列表
     * @param unassigned_objects_idx  需要创建轨迹的传感器数据下标
     */
    void CreateNewTracks(const SensorFrame& sensor_list, const std::vector<uint32_t>& unassigned_objects_idx);

    /**
     * @brief 移除已失效的轨迹跟踪器
     */
    void RemoveLostTracks();

    /**
     * @brief 融合指向同一目标轨迹的轨迹跟踪器
     * @param type 当前帧数据传感器类型
     */
    void FuseSameTracks(const SensorType type);

    uint64_t GetCurrentTime();

private:
    TrackPredictor  predictor;
    DataAssociation matcher;
    TrackUpdater    updater;
};

#endif /* C1E75B9A_A0C0_4323_B95D_5C0A6FD95686 */
