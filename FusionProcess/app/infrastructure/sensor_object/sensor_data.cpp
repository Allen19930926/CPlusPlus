#include "sensor_data.h"
#include <glog/logging.h>

namespace
{
    const uint8_t  maxCachedFrameNum = 10;
}

void Sensor::QueryLatestFrame(std::vector<SensorFrame>& queryFrames)
{
    if (frames_.size() == 0)
    {
        LOG(INFO) << "there is no frame data, get frame fail";
        return ;
    }

    auto latest_rule = [](const SensorFrame& frame1, const SensorFrame& frame2)
                                {
                                    return frame1.time_stamp < frame2.time_stamp;
                                };

    auto iter = std::max_element(frames_.begin(), frames_.end(), latest_rule);
    SensorFrame& latest_frame = *iter;

    // 有可能融合周期内没有新的传感器数据，取到的是上一周期使用的传感器帧，这种情况应该获取失败
    if (latest_frame.is_fused == 0)
    {
        latest_frame.is_fused = 1;
        queryFrames.push_back(latest_frame);
        latest_fuse_time = latest_frame.time_stamp;
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
    latest_fuse_time = 0;
}
