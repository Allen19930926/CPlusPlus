#include "sensor_data_manager.h"
#include <unordered_map>
#include <glog/logging.h>

SensorDataManager::SensorDataManager()
{
    sensors.emplace(std::make_pair(SensorType::CAMERA      ,   Sensor(SensorType::CAMERA)));
    sensors.emplace(std::make_pair(SensorType::V2X         ,   Sensor(SensorType::V2X)));
    sensors.emplace(std::make_pair(SensorType::FRONT_RADAR ,   Sensor(SensorType::FRONT_RADAR)));
}

SensorDataManager::~SensorDataManager()
{
    sensors.clear();
}

void SensorDataManager::AddSensorMeasurements(const SensorFrame& frame)
{
    if (!IsTargetSensor(frame))
    {
        LOG(WARNING) << "Add sensor measurement fail";
        return ;
    }

    auto pair = sensors.find(frame.sensor_type);
    pair->second.AddFrame(frame);
}

void SensorDataManager::QueryLatestFrames(const uint32_t time_stamp, std::vector<SensorFrame>& frames)
{
    frames.clear();

    for (auto& pair : sensors)
    {
        std::vector<SensorFrame> eachSensorFrames;
        pair.second.QueryLatestFrame(time_stamp, eachSensorFrames);
        frames.insert(frames.end(), eachSensorFrames.begin(), eachSensorFrames.end());
    }

    std::sort(frames.begin(), frames.end(), [](const SensorFrame& lhs, const SensorFrame& rhs) { return lhs.time_stamp < rhs.time_stamp; });

}

uint32_t SensorDataManager::GetCacheFrameNum(const SensorType sensor_type)
{
    uint32_t count = 0;
    auto pair = sensors.find(sensor_type);
    if (pair != sensors.end())
    {
        count = pair->second.GetCachedFrameNum();
    }
    return count;
}

void SensorDataManager::Clear()
{
    for (auto& pair : sensors)
    {
        pair.second.Clear();
    }
}

bool SensorDataManager::IsTargetSensor(const SensorFrame& frame)
{
    return (sensors.find(frame.sensor_type) != sensors.end());
}

