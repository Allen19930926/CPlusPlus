#include "sensor_data.h"
#include <glog/logging.h>

namespace
{
    const uint8_t  maxCachedFrameNum = 10;
}

bool Sensor::QueryLatestFrame(const uint64_t timeStamp, std::vector<SensorFrame>& queryFrames)
{
    if (frames_.size() == 0)
    {
        LOG(INFO) << "there is no frame data, get frame fail";
        return false;
    }

    for (uint32_t i=0; i<frames_.size(); i++)
    {
        if (frames_[i].timeStamp > timeStamp || frames_[i].timeStamp <= latestQueryTime)
        {
            continue;
        }
        queryFrames.push_back(frames_[i]);
    }

    latestQueryTime = timeStamp;

    return true;
}

// 队列保持先进先出，不一定准确按照时间排序。当队列已满时，先抛出对头元素（大概率是较早时间的数据），再添加本帧元素
void Sensor::AddFrame(const SensorFrame& frame)
{
    if (frames_.size() >= maxCachedFrameNum)
    {
        frames_.pop_front();
    }
    frames_.emplace_back(frame);
}

