#include "gtest/gtest.h"
#include <glog/logging.h>
#include "sensor_data_manager.h"


struct SensorDataManagerTest : testing::Test
{
    void SetUp()
    {
        manager = &SensorDataManager::GetInstance();
        manager->Clear();
    }

    void TearDown()
    {
        manager->Clear();
        manager = nullptr;
    }

    SensorDataManager* manager;
};

TEST_F(SensorDataManagerTest, add_frame_normal_test)
{
    SensorFrame frame;
    frame.sensorType = 0;
    frame.timeStamp = 13487432;
    manager->AddSensorMeasurements(frame);

    frame.sensorType = 1;
    manager->AddSensorMeasurements(frame);

    frame.sensorType = 2;
    manager->AddSensorMeasurements(frame);

    ASSERT_EQ(uint32_t(1), manager->GetCacheFrameNum("camera"));
    ASSERT_EQ(uint32_t(1), manager->GetCacheFrameNum("v2x"));
    ASSERT_EQ(uint32_t(1), manager->GetCacheFrameNum("front_radar"));
}

TEST_F(SensorDataManagerTest, add_frame_fail_and_target_frame_judge_test)
{
    SensorFrame frame;
    frame.sensorType = 255;
    frame.timeStamp = 13487432;
    manager->AddSensorMeasurements(frame);

    frame.sensorType = 15;
    manager->AddSensorMeasurements(frame);

    ASSERT_EQ(uint32_t(0), manager->GetCacheFrameNum("camera"));
    ASSERT_EQ(uint32_t(0), manager->GetCacheFrameNum("v2x"));
    ASSERT_EQ(uint32_t(0), manager->GetCacheFrameNum("front_radar"));
}

TEST_F(SensorDataManagerTest, query_lastest_frames_test)
{
    SensorFrame frame;
    frame.sensorType = 0;
    frame.timeStamp = 13487432;
    for (uint32_t i=0; i<20; i++)
    {
        frame.timeStamp++;
        manager->AddSensorMeasurements(frame);
    }
    frame.timeStamp = 1000;
    for (uint32_t i=0; i<4; i++)
    {
        frame.timeStamp -= 200;
        manager->AddSensorMeasurements(frame);
    }

    ASSERT_EQ(uint32_t(10), manager->GetCacheFrameNum("camera"));

    std::vector<SensorFrame> res;
    manager->QueryLatestFrames(800, res);
    ASSERT_EQ(uint32_t(4), res.size());

    res.clear();
    manager->QueryLatestFrames(0xFFFFFFFF, res);
    ASSERT_EQ(uint32_t(6), res.size());

    res.clear();
    manager->QueryLatestFrames(800, res);
    ASSERT_EQ(uint32_t(0), res.size());
}

TEST_F(SensorDataManagerTest, get_cached_frame_num_test)
{
    SensorFrame frame;
    frame.sensorType = 0;
    frame.timeStamp = 13487432;
    manager->AddSensorMeasurements(frame);

    ASSERT_EQ(uint32_t(1), manager->GetCacheFrameNum("camera"));
    ASSERT_EQ(uint32_t(0), manager->GetCacheFrameNum("rfyrthtgrf"));
}


