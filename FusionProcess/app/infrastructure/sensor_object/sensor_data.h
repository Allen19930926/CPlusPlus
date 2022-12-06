#ifndef BAE45A8F_0A87_4B55_9C11_29442A2C3B5D
#define BAE45A8F_0A87_4B55_9C11_29442A2C3B5D

#include "sensor_object.h"
#include <queue>
#include <vector>

class Sensor
{
public:
    Sensor(const SensorType type): sensor_type(type), latest_fuse_time(0) {}

    /**
     * @brief 获取该传感器类型的最近一帧数据
     * @param time_stamp 截止时间戳
     * @param queryFrames 获取的结果
     */
    void QueryLatestFrame(std::vector<SensorFrame>& queryFrames);

    /**
     * @brief 添加传感器数据帧，如果缓存数量超过门限，则丢弃最旧的数据帧，再进行添加
     * @param frame 传感器数据帧
     */
    void AddFrame(const SensorFrame& frame);
    
    /**
     * @brief 获取已缓存传感器数据帧数量
     * @return 缓存数量
     */
    uint32_t GetCachedFrameNum() {return frames_.size();}

    /**
     * @brief 置0操作，供测试用例使用
     */
    void Clear();
private:
    SensorType  sensor_type;
    uint32_t    latest_fuse_time;
    std::deque<SensorFrame> frames_;
};

#endif /* BAE45A8F_0A87_4B55_9C11_29442A2C3B5D */
