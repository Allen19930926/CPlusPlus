#include "gtest/gtest.h"
#include <glog/logging.h>

#include "infrastructure/common/fusion_tools.h"
#include "infrastructure/sensor_object/sensor_data_manager.h"

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

TEST_F(SensorDataManagerTest, add_frame_success_with_differenr_type_test)
{
    SensorFrame frame;
    SensorObject null_obj;
    frame.sensor_type = SensorType::CAMERA;
    frame.time_stamp = 13487432;
    frame.is_fused = 0;
    frame.sensors.push_back(null_obj);
    manager->AddSensorMeasurements(frame);

    frame.sensor_type = SensorType::V2X;
    manager->AddSensorMeasurements(frame);

    frame.sensor_type = SensorType::FRONT_RADAR;
    manager->AddSensorMeasurements(frame);

    ASSERT_EQ(uint32_t(1), manager->GetCacheFrameNum(SensorType::CAMERA));
    ASSERT_EQ(uint32_t(1), manager->GetCacheFrameNum(SensorType::V2X));
    ASSERT_EQ(uint32_t(1), manager->GetCacheFrameNum(SensorType::FRONT_RADAR));
}

TEST_F(SensorDataManagerTest, add_frame_success_with_differenr_time_test)
{
    SensorFrame frame;
    SensorObject null_obj;
    frame.sensor_type = SensorType::CAMERA;
    frame.time_stamp = 13487432;
    frame.is_fused = 0;
    frame.sensors.push_back(null_obj);
    manager->AddSensorMeasurements(frame);

    frame.time_stamp = 13487433;
    manager->AddSensorMeasurements(frame);

    ASSERT_EQ(uint32_t(2), manager->GetCacheFrameNum(SensorType::CAMERA));
}

TEST_F(SensorDataManagerTest, add_frame_fail_and_target_frame_judge_test)
{
    SensorFrame frame;
    SensorObject null_obj;
    frame.sensor_type = SensorType::MAX;
    frame.time_stamp = 13487432;
    frame.is_fused = 0;
    frame.sensors.push_back(null_obj);
    manager->AddSensorMeasurements(frame);

    ASSERT_EQ(uint32_t(0), manager->GetCacheFrameNum(SensorType::CAMERA));
    ASSERT_EQ(uint32_t(0), manager->GetCacheFrameNum(SensorType::V2X));
    ASSERT_EQ(uint32_t(0), manager->GetCacheFrameNum(SensorType::FRONT_RADAR));
}

TEST_F(SensorDataManagerTest, query_lastest_frames_with_different_time_test)
{
    SensorFrame frame;
    SensorObject null_obj;
    frame.sensor_type = SensorType::CAMERA;
    frame.sensors.push_back(null_obj);
    frame.time_stamp = 10000;
    frame.is_fused = 0;
    for (uint32_t i = 0; i < 20; i++)
    {
        frame.time_stamp -= 200;
        manager->AddSensorMeasurements(frame);
    }
    std::vector<SensorFrame> res;
    manager->QueryLatestFrames(res);
    ASSERT_EQ(uint32_t(1), res.size());
    ASSERT_EQ(uint32_t(7800), res[0].time_stamp);
    ASSERT_EQ(uint32_t(10), manager->GetCacheFrameNum(SensorType::CAMERA));

    frame.time_stamp = 13487432;
    for (uint32_t i = 0; i < 5; i++)
    {
        frame.time_stamp++;
        manager->AddSensorMeasurements(frame);
    }

    res.clear();
    manager->QueryLatestFrames(res);
    ASSERT_EQ(uint32_t(1), res.size());
    ASSERT_EQ(frame.time_stamp, res[0].time_stamp);

}

TEST_F(SensorDataManagerTest, query_lastest_frames_with_different_type_test)
{
    SensorFrame frame;
    SensorObject null_obj;

    // camera frame: 20, 21, 22, 23, 24, 25, 26, 27, 28, 29
    frame.sensor_type = SensorType::CAMERA;
    frame.time_stamp = 10;
    frame.is_fused = 0;
    frame.sensors.push_back(null_obj);
    for (uint32_t i = 0; i < 20; i++, frame.time_stamp++)
    {
        manager->AddSensorMeasurements(frame);
    }

    // v2x frame: 25, 27, 29, 31
    frame.sensor_type = SensorType::V2X;
    frame.time_stamp = 25;
    for (uint32_t i = 0; i < 4; i++, frame.time_stamp += 2)
    {
        manager->AddSensorMeasurements(frame);
    }

    ASSERT_EQ(uint32_t(10), manager->GetCacheFrameNum(SensorType::CAMERA));

    std::vector<SensorFrame> res;
    manager->QueryLatestFrames(res);

    // res expect: 25(c), 25(v)
    ASSERT_EQ(uint32_t(2), res.size());
    ASSERT_EQ(SensorType::CAMERA, res[0].sensor_type);
    ASSERT_EQ(SensorType::V2X,    res[1].sensor_type);
}

TEST_F(SensorDataManagerTest, get_cached_frame_num_test)
{
    SensorFrame frame;
    SensorObject null_obj;
    frame.sensor_type = SensorType::CAMERA;
    frame.time_stamp = 13487432;
    frame.is_fused = 0;
    frame.sensors.push_back(null_obj);
    manager->AddSensorMeasurements(frame);

    ASSERT_EQ(uint32_t(1), manager->GetCacheFrameNum(SensorType::CAMERA));
    ASSERT_EQ(uint32_t(0), manager->GetCacheFrameNum(SensorType::MAX));
}

TEST_F(SensorDataManagerTest, 10000_10001_query_frames_with_no_cached_frames_test)
{
    std::vector<SensorFrame> frames;
    manager->QueryLatestFrames(frames);
    ASSERT_EQ(uint32_t(0), frames.size());
}

TEST_F(SensorDataManagerTest, 10004_query_frames_with_no_cached_object_test)
{
    SensorFrame frame;
    frame.sensor_type = SensorType::CAMERA;
    frame.time_stamp = 13487432;
    frame.is_fused = 0;
    manager->AddSensorMeasurements(frame);

    std::vector<SensorFrame> frames;
    manager->QueryLatestFrames(frames);
    ASSERT_EQ(uint32_t(1), frames.size());
    ASSERT_EQ(uint32_t(0), frames[0].sensors.size());
}

TEST_F(SensorDataManagerTest, 10003_10005_query_frames_with_error_object_attr_test)
{
    SensorFrame frame;
    SensorObject obj;
    frame.sensor_type = SensorType::CAMERA;
    frame.time_stamp = 13487432;
    frame.is_fused = 0;
    obj.position << 0xffffffff, 0xffffffff;
    frame.sensors.push_back(obj);
    manager->AddSensorMeasurements(frame);

    std::vector<SensorFrame> frames;
    manager->QueryLatestFrames(frames);
    ASSERT_EQ(uint32_t(1), frames.size());
    ASSERT_EQ(uint32_t(1), frames[0].sensors.size());
}

