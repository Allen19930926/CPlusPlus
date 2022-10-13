#ifndef FA3A08FF_9ADF_49BD_9FDD_336756F0C840
#define FA3A08FF_9ADF_49BD_9FDD_336756F0C840

#include "sensor_data.h"
#include <string>
#include <unordered_map>
#include "gtest/gtest.h"

class SensorDataManager
{
public:
    static SensorDataManager& GetInstance()
    {
        static SensorDataManager manager;
        return manager;
    }
    void AddSensorMeasurements(const SensorFrame& frame);
    bool QueryLatestFrames(const uint32_t time_stamp, std::vector<SensorFrame>& frames);
    uint32_t GetCacheFrameNum(const SensorType sensorName);
    void Clear();
    bool IsTargetSensor(const SensorFrame& frame);

private:
    SensorDataManager();
    ~SensorDataManager();

public:
    std::unordered_map<SensorType, Sensor> sensors;
};

#endif /* FA3A08FF_9ADF_49BD_9FDD_336756F0C840 */
