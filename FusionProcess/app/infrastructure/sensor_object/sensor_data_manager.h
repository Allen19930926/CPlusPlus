#ifndef FA3A08FF_9ADF_49BD_9FDD_336756F0C840
#define FA3A08FF_9ADF_49BD_9FDD_336756F0C840

#include "sensor_data.h"
#include <string>
#include <unordered_map>

class SensorDataManager
{
public:
    static SensorDataManager& GetInstance()
    {
        static SensorDataManager manager;
        return manager;
    }
    /**
     * @brief 添加当前帧所有的传感器数据到SensorManager
     * @param frame 待添加的传感器数据帧
     */
    void AddSensorMeasurements(const SensorFrame& frame);

    /**
     * @brief 获取所有传感器类型的最近一帧数据，按照时间和类型排序
     * @param time_stamp 截止时间戳
     * @param frames 获取的结果
     */
    void QueryLatestFrames(const uint32_t time_stamp, std::vector<SensorFrame>& frames);

    /**
     * @brief 查询某类传感器缓存的数据帧数量
     * @param sensorName 查询的传感器类型
     * @return 缓存数量
     */
    uint32_t GetCacheFrameNum(const SensorType sensorName);

    /**
     * @brief 置0操作，供测试用例使用
     */
    void Clear();

private:
    /**
     * @brief 查询该传感器类型，是否为可缓存类型
     * @param frame 查询的传感器类型
     * @return 查询结果
     */
    bool IsTargetSensor(const SensorFrame& frame);
    SensorDataManager();
    ~SensorDataManager();

public:
    bool recv_new_data;
    std::unordered_map<SensorType, Sensor> sensors;
};

#endif /* FA3A08FF_9ADF_49BD_9FDD_336756F0C840 */
