#include "sensor_data.h"
#include <glog/logging.h>

namespace
{
    const uint8_t  maxCachedFrameNum = 10;
}

void Sensor::QueryLatestFrame(const uint32_t time_stamp, std::vector<SensorFrame>& queryFrames)
{
    if (frames_.size() == 0)
    {
        LOG(INFO) << "there is no frame data, get frame fail";
        return ;
    }

    for (uint32_t i=0; i<frames_.size(); i++)
    {
        if (frames_[i].time_stamp > time_stamp || frames_[i].time_stamp <= latestQueryTime)
        {
            continue;
        }
        queryFrames.push_back(frames_[i]);
        latestQueryTime = std::max(latestQueryTime, frames_[i].time_stamp);
    }
}

// 队列保持先进先出，不一定准确按照时间排序。当队列已满时，先抛出队头元素（大概率是较早时间的数据），再添加本帧元素
void Sensor::AddFrame(const SensorFrame& frame)
{
    if (sensor_type != frame.sensor_type)
    {
        LOG(WARNING) << "The frame(" << static_cast<uint32_t>(frame.sensor_type)
                     << ") is not the same type(" << static_cast<uint32_t>(sensor_type) << ")";
        return ;
    }

    // 理论上使用多态更符合编码原则，但是这里改动会比较大，先插入一段只属于camera的合并流程
    if (sensor_type == SensorType::CAMERA)
    {
        if (!frames_.empty())
        {
            SensorFrame& last_frame = frames_.back();
            if (last_frame.time_stamp == frame.time_stamp)
            {
                last_frame.sensors.insert(last_frame.sensors.end(), frame.sensors.begin(), frame.sensors.end());
                return ;
            }
        }
    }

    if (frames_.size() >= maxCachedFrameNum)
    {
        frames_.pop_front();
    }
    frames_.emplace_back(frame);
}

void Sensor::Clear()
{
    frames_.clear();
    latestQueryTime = 0;
}
