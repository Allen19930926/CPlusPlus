#include "sensor_data_manager.h"
#include <unordered_map>
#include <glog/logging.h>

namespace
{
    std::unordered_map<std::string, uint8_t> sensorNameReflex  =  { {"camera"     , 0},
                                                                    {"v2x"        , 1},
                                                                    {"front_radar", 2},};

    std::unordered_map<uint8_t, std::string> sensorTypeReflex  =  { {0,   "camera"     },
                                                                    {1,   "v2x"        },
                                                                    {2,   "front_radar"},
                                                                    {255, "for_test"   }};
}

SensorDataManager::SensorDataManager()
{
    sensors.emplace(std::make_pair("camera"     ,   Sensor(sensorNameReflex["camera"])));
    sensors.emplace(std::make_pair("v2x"        ,   Sensor(sensorNameReflex["v2x"])));
    sensors.emplace(std::make_pair("front_radar",   Sensor(sensorNameReflex["front_radar"])));
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

    auto pair = sensors.find(sensorTypeReflex[frame.sensorType]);
    if (pair != sensors.end())
    {
        pair->second.AddFrame(frame);
    }
}

bool SensorDataManager::QueryLatestFrames(const uint32_t timeStamp, std::vector<SensorFrame>& frames)
{
    frames.clear();

    for (auto& pair : sensors)
    {
        std::vector<SensorFrame> eachSensorFrames;
        pair.second.QueryLatestFrame(timeStamp, eachSensorFrames);
        frames.insert(frames.end(), eachSensorFrames.begin(), eachSensorFrames.end());
    }

    std::sort(frames.begin(), frames.end(), [](const SensorFrame& lhs, const SensorFrame& rhs) { return lhs.timeStamp < rhs.timeStamp; });

    return true;
}

uint32_t SensorDataManager::GetCacheFrameNum(const SensorType sensorName)
{
    uint32_t count = 0;
    auto pair = sensors.find(sensorName);
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
    if (sensorTypeReflex.find(frame.sensorType) == sensorTypeReflex.end())
    {
        LOG(WARNING) << "invalid sensor typeid: " << frame.sensorType;
        return false;
    }

    auto pair = sensors.find(sensorTypeReflex[frame.sensorType]);
    if (pair == sensors.end())
    {
        LOG(WARNING) << "This sensor is not cached, typeid: " << frame.sensorType;
        return false;
    }
    return true;
}

