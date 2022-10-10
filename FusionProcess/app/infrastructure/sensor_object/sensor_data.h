#ifndef BAE45A8F_0A87_4B55_9C11_29442A2C3B5D
#define BAE45A8F_0A87_4B55_9C11_29442A2C3B5D

#include "sensor_object.h"
#include <queue>
#include "gtest/gtest.h"

class SensorDataTest_add_max_frame_test;

class Sensor
{
public:
    Sensor(const uint8_t type): sensorType(type) {}
    bool QueryNeareatFrame(const uint64_t timeStamp, SensorFrame& queryFrame);
    void AddFrame(const SensorFrame& frame);
    bool IsSpecificType(const uint8_t type) {return type == sensorType;}
private:
    uint32_t GetCachedFrameNum() {return frames_.size();}
private:
    FRIEND_TEST(SensorDataTest, add_max_frame_test);
    uint8_t     sensorType;
    std::deque<SensorFrame> frames_;
};



#endif /* BAE45A8F_0A87_4B55_9C11_29442A2C3B5D */
