#include "gtest/gtest.h"
#include <glog/logging.h>
#include <algorithm>

#include "infrastructure/sensor_object/sensor_data.h"

struct SensorDataTest : testing::Test
{
    void SetUp()
    {
        sensor = new Sensor(SensorType::CAMERA);
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
    frame.sensor_type = SensorType::CAMERA;
    frame.time_stamp = 12468431;
    frame.is_fused = 0;

    for (uint32_t i = 0; i < 50; i++)
    {
        frame.time_stamp++;
        sensor->AddFrame(frame);
    }

    ASSERT_EQ(uint32_t(10), sensor->GetCachedFrameNum());

    std::vector<SensorFrame> queryFrame;
    sensor->QueryLatestFrame(queryFrame);
    ASSERT_EQ(uint32_t(1), queryFrame.size());
    ASSERT_EQ(frame.time_stamp, queryFrame[0].time_stamp);
}

TEST_F(SensorDataTest, add_frame_fail_test)
{
    SensorFrame frame;
    frame.sensor_type = SensorType::MAX;
    frame.time_stamp = 12468431;
    frame.is_fused = 0;

    sensor->AddFrame(frame);

    ASSERT_EQ(uint32_t(0), sensor->GetCachedFrameNum());
}

TEST_F(SensorDataTest, query_frame_with_null_input_test)
{
    std::vector<SensorFrame> queryFrame;
    sensor->QueryLatestFrame(queryFrame);

    ASSERT_EQ(uint32_t(0), queryFrame.size());
}

TEST_F(SensorDataTest, query_frame_normal_test)
{
    SensorFrame frame;
    frame.sensor_type = SensorType::CAMERA;
    frame.time_stamp = 12468431;
    frame.is_fused = 0;

    for (uint32_t i = 0; i < 50; i++)
    {
        frame.time_stamp += 2;
        sensor->AddFrame(frame);
    }

    uint32_t latestTime = frame.time_stamp;

    frame.time_stamp = 1000;
    for (uint32_t i = 0; i < 5; i++)
    {
        frame.time_stamp -= 100;
        sensor->AddFrame(frame);
    }

    std::vector<SensorFrame> queryFrame;

    // deque   12468523 12468525 12468527 12468529 12468531 1000 900 800 700 600
    // expect  12468531
    sensor->QueryLatestFrame(queryFrame);
    ASSERT_EQ(uint32_t(1), queryFrame.size());
    ASSERT_EQ(uint32_t(12468531), queryFrame[0].time_stamp);
}

TEST_F(SensorDataTest, clear_frame_test)
{
    SensorFrame frame;
    frame.sensor_type = SensorType::CAMERA;
    frame.time_stamp = 12468431;

    for (uint32_t i = 0; i < 50; i++)
    {
        frame.time_stamp++;
        sensor->AddFrame(frame);
    }
    sensor->Clear();

    ASSERT_EQ(uint32_t(0), sensor->GetCachedFrameNum());
}

TEST_F(SensorDataTest, Add_camera_vehicle_and_pedestrain_frame_with_same_timestamp_test)
{
    SensorFrame frame;
    frame.sensor_type = SensorType::CAMERA;
    frame.time_stamp = 12468431;
    SensorObject vehi;
    vehi.id = 25;
    frame.sensors.push_back(vehi);
    sensor->AddFrame(frame);
    ASSERT_EQ(uint32_t(1), sensor->GetCachedFrameNum());

    frame.sensors.clear();
    SensorObject pedes;
    pedes.id = 20;
    frame.sensors.push_back(pedes);
    sensor->AddFrame(frame);

    ASSERT_EQ(uint32_t(1), sensor->GetCachedFrameNum());
}
