#include "gtest/gtest.h"
#include "sensor_data.h"
#include <glog/logging.h>

struct SensorDataTest : testing::Test
{
    void SetUp()
    {
        sensor = new Sensor(1);
    }

    void TearDown()
    {
        delete sensor;
    }

    Sensor* sensor;
};

TEST_F(SensorDataTest, add_max_frame_test)
{
    SensorFrame frame;
    frame.sensorType = 1;
    frame.timeStamp = 12468431;

    for (uint32_t i=0; i<50; i++)
    {
        frame.timeStamp++;
        sensor->AddFrame(frame);
    }

    EXPECT_EQ(uint32_t(10), sensor->GetCachedFrameNum());

    SensorFrame queryFrame;
    for (uint32_t i=0; i<10; i++)
    {
        EXPECT_TRUE(sensor->QueryNeareatFrame(frame.timeStamp, queryFrame));
        EXPECT_EQ(frame.timeStamp, queryFrame.timeStamp);
        frame.timeStamp--;
    }
}

TEST_F(SensorDataTest, query_frame_with_null_deque_test)
{
    SensorFrame frame;
    frame.sensorType = 1;
    frame.timeStamp = 12468431;

    SensorFrame queryFrame;

    EXPECT_FALSE(sensor->QueryNeareatFrame(frame.timeStamp, queryFrame));
}

TEST_F(SensorDataTest, query_frame_normal_test)
{
    SensorFrame frame;
    frame.sensorType = 1;
    frame.timeStamp = 12468431;

    for (uint32_t i=0; i<50; i++)
    {
        frame.timeStamp += 2;
        sensor->AddFrame(frame);
    }

    uint32_t latestTime = frame.timeStamp;

    frame.timeStamp = 1000;
    for (uint32_t i=0; i<5; i++)
    {
        frame.timeStamp -= 100;
        sensor->AddFrame(frame);
    }

    SensorFrame queryFrame;

    // deque   12468523 12468525 12468527 12468529 12468531 1000 900 800 700 600
    // query   12468530
    // expect  12468529
    EXPECT_TRUE(sensor->QueryNeareatFrame(latestTime - 1, queryFrame));
    EXPECT_EQ(latestTime-2, queryFrame.timeStamp);
}
