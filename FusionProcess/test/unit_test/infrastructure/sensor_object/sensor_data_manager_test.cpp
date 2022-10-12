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
    frame.sensorType = SensorType::CAMERA;
    frame.timeStamp = 13487432;
    manager->AddSensorMeasurements(frame);

    frame.sensorType = SensorType::V2X;
    manager->AddSensorMeasurements(frame);

    frame.sensorType = SensorType::FRONT_RADAR;
    manager->AddSensorMeasurements(frame);

    ASSERT_EQ(uint32_t(1), manager->GetCacheFrameNum(SensorType::CAMERA));
    ASSERT_EQ(uint32_t(1), manager->GetCacheFrameNum(SensorType::V2X));
    ASSERT_EQ(uint32_t(1), manager->GetCacheFrameNum(SensorType::FRONT_RADAR));
}

TEST_F(SensorDataManagerTest, add_frame_fail_and_target_frame_judge_test)
{
    SensorFrame frame;
    frame.sensorType = SensorType::INVALID;
    frame.timeStamp = 13487432;
    manager->AddSensorMeasurements(frame);

    ASSERT_EQ(uint32_t(0), manager->GetCacheFrameNum(SensorType::CAMERA));
    ASSERT_EQ(uint32_t(0), manager->GetCacheFrameNum(SensorType::V2X));
    ASSERT_EQ(uint32_t(0), manager->GetCacheFrameNum(SensorType::FRONT_RADAR));
}

TEST_F(SensorDataManagerTest, query_lastest_frames_test)
{
    SensorFrame frame;
    frame.sensorType = SensorType::CAMERA;
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

    ASSERT_EQ(uint32_t(10), manager->GetCacheFrameNum(SensorType::CAMERA));

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
    frame.sensorType = SensorType::CAMERA;
    frame.timeStamp = 13487432;
    manager->AddSensorMeasurements(frame);

    ASSERT_EQ(uint32_t(1), manager->GetCacheFrameNum(SensorType::CAMERA));
    ASSERT_EQ(uint32_t(0), manager->GetCacheFrameNum(SensorType::INVALID));
}

// TEST_F(SensorDataManagerTest, any_test)
// {
//     FusionTrackKfData kf;
//     kf.x_prior << 1,2,3,4,5,6;
//     kf.p_prior << 1,2,3,4,5,6,
//                   2,4,6,8,10,12,
//                   3,6,9,12,15,18,
//                   4,8,12,16,20,24,
//                   5,10,15,20,25,30,
//                   6,12,18,24,30,36;
//     LOG(INFO) << '\n' << kf.x_prior;
//     LOG(INFO) << '\n' << kf.p_prior;
// }


