#include "sensor_data_manager.h"
#include <unordered_map>
#include <glog/logging.h>
#include <mutex>

namespace
{
    std::mutex sensor_lck;
}


SensorDataManager::SensorDataManager() : recv_new_data(false)
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

    std::lock_guard<std::mutex> lck_guard(sensor_lck);
    recv_new_data = true;
    auto pair = sensors.find(frame.sensor_type);
    pair->second.AddFrame(frame);
}

void SensorDataManager::QueryLatestFrames(std::vector<SensorFrame>& frames)
{
    frames.clear();

    {
        std::lock_guard<std::mutex> lck_guard(sensor_lck);
        if (!recv_new_data)
        {
            return ;
        }
        recv_new_data = false;
        for (auto& pair : sensors)
        {
            std::vector<SensorFrame> eachSensorFrames;
            pair.second.QueryLatestFrame(eachSensorFrames);
            frames.insert(frames.end(), eachSensorFrames.begin(), eachSensorFrames.end());
        }
    }

    auto sensor_frame_compare = [](const SensorFrame& lhs, const SensorFrame& rhs)
                                {
                                        if (lhs.time_stamp != rhs.time_stamp)
                                        {
                                            return lhs.time_stamp < rhs.time_stamp; 
                                        }
                                        return lhs.sensor_type < rhs.sensor_type; 
                                };

    std::sort(frames.begin(), frames.end(), sensor_frame_compare);

	for (auto& it : frames)
	{
		LOG(INFO) << it.time_stamp << " : " << it.sensor_type;
	}

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

