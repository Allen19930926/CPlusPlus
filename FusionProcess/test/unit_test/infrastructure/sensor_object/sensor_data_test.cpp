#include "gtest/gtest.h"
#include "sensor_data.h"
#include <glog/logging.h>
#include <algorithm>

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

    std::vector<SensorFrame> queryFrame;
    sensor->QueryLatestFrame(frame.timeStamp, queryFrame);
    std::sort(queryFrame.begin(), queryFrame.end(), [](const SensorFrame& lhs, const SensorFrame& rhs) {return lhs.timeStamp > rhs.timeStamp;});

    for (const auto& it : queryFrame)
    {
        EXPECT_EQ(frame.timeStamp, it.timeStamp);
        frame.timeStamp--;
    }
}

TEST_F(SensorDataTest, query_frame_with_null_deque_test)
{
    SensorFrame frame;
    frame.sensorType = 1;
    frame.timeStamp = 12468431;

    std::vector<SensorFrame> queryFrame;

    EXPECT_FALSE(sensor->QueryLatestFrame(frame.timeStamp, queryFrame));
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

    std::vector<SensorFrame> queryFrame;
    EXPECT_TRUE(sensor->QueryLatestFrame(1000, queryFrame));
    queryFrame.clear();

    // deque   12468523 12468525 12468527 12468529 12468531 1000 900 800 700 600
    // query   12468530
    // expect  12468529
    EXPECT_TRUE(sensor->QueryLatestFrame(latestTime - 1, queryFrame));
    EXPECT_EQ(uint32_t(4), queryFrame.size());

    std::sort(queryFrame.begin(), queryFrame.end(), [](const SensorFrame& lhs, const SensorFrame& rhs) {return lhs.timeStamp > rhs.timeStamp;});

    for (const auto& it : queryFrame)
    {
        latestTime -= 2;
        EXPECT_EQ(latestTime, it.timeStamp);
    }
}
