#ifndef BAE45A8F_0A87_4B55_9C11_29442A2C3B5D
#define BAE45A8F_0A87_4B55_9C11_29442A2C3B5D

#include "sensor_object.h"
#include <queue>
#include <vector>

class Sensor
{
public:
    /// @brief constructor func
    /// @param type 
    Sensor(const SensorType type): sensor_type(type), latestQueryTime(0) {}
    /// @brief query and get sensor frames since latestQueryTime
    /// @param time_stamp 
    /// @param queryFrames
    void QueryLatestFrame(const uint32_t time_stamp, std::vector<SensorFrame>& queryFrames);
    /// @brief Add new frame to deque. if cached frame number is greater than maxCachedFrameNum , then will drop oldest frame data
    /// @param frame 
    void AddFrame(const SensorFrame& frame);
    /// @brief judge whether class type is the same as input sensor type
    /// @param type 
    /// @return whether class type is the same as input sensor type
    bool IsSpecificType(const SensorType type) {return type == sensor_type;}
    uint32_t GetCachedFrameNum() {return frames_.size();}
    void Clear() {frames_.clear();}
private:
    SensorType  sensor_type;
    uint32_t    latestQueryTime;
    std::deque<SensorFrame> frames_;
};

#endif /* BAE45A8F_0A87_4B55_9C11_29442A2C3B5D */
