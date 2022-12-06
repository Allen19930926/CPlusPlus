#include "application/fuse_system/fusion_system.h"
#include "infrastructure/track_object/fusion_track_manager.h"
#include "infrastructure/common/ipc_data.h"
#include "infrastructure/common/fusion_tools.h"
#include "infrastructure/sensor_object/sensor_data_manager.h"

#include "gtest/gtest.h"
#include "stub/stub.h"
#include "stub/addr_pri.h"


struct FusionSystemTest : testing::Test
{
    void TearDown() override
    {
        FusionTrackManager::GetInstance().Clear();
        SensorDataManager::GetInstance().Clear();
    }

    void Fuse()
    {
        sys.Fuse();
    }


public:
    FusionSystem sys;
};



TEST_F(FusionSystemTest, fuse_with_empty_input_test)
{
    Fuse();
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    ASSERT_EQ(uint32_t(0), track_list.size());

}

TEST_F(FusionSystemTest, create_one_track_test)
{
    SensorFrame frame;
    SensorObject obj;
    frame.sensor_type = SensorType::V2X;
    frame.is_fused = 0;
    obj.time_stamp = 1;
    obj.life_time = 2;
    obj.id = 25;
    obj.position << 0, 20;
    obj.velocity << 0, 0;
    obj.acceleration << 0, 0;
    obj.pos_variance << 1, 0, 0, 1;
    obj.vel_variance << 1, 0, 0, 1;
    obj.acc_variance << 1, 0, 0, 1;
    obj.sensor_type = 1;
    frame.time_stamp = obj.time_stamp;
    frame.sensors.push_back(obj);
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);

    Fuse();
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    ASSERT_EQ(uint32_t(1), track_list.size());

    const FusionTrack& track = track_list[0];
    ASSERT_EQ(obj.id, track.sensor_trajetories[1].sensor_id);
    ASSERT_EQ(obj.position(0), track.position(0));
    ASSERT_EQ(obj.position(1), track.position(1));
}

TEST_F(FusionSystemTest, keep_one_track_by_id_test)
{
    SensorFrame frame;
    SensorObject obj;
    frame.sensor_type = SensorType::V2X;
    frame.is_fused = 0;
    obj.time_stamp = 1;
    obj.life_time = 2;
    obj.id = 25;
    obj.position << 0, 20;
    obj.velocity << 20, 0;
    obj.acceleration << 0, 0;
    obj.pos_variance << 1, 0, 0, 1;
    obj.vel_variance << 1, 0, 0, 1;
    obj.acc_variance << 1, 0, 0, 1;
    obj.sensor_type = 1;
    frame.time_stamp = obj.time_stamp;
    frame.sensors.push_back(obj);
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);

    Fuse();
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    ASSERT_EQ(uint32_t(1), track_list.size());

    const FusionTrack& track = track_list[0];
    ASSERT_EQ(obj.id, track.sensor_trajetories[1].sensor_id);
    ASSERT_EQ(obj.position(0), track.position(0));
    ASSERT_EQ(obj.position(1), track.position(1));

    frame.sensors.clear();
    obj.time_stamp = 2;
    obj.life_time = 3;
    obj.position << 20, 20;
    frame.time_stamp = obj.time_stamp;
    frame.sensors.push_back(obj);
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);
    Fuse();
    ASSERT_EQ(uint32_t(1), track_list.size());

    ASSERT_EQ(obj.id, track.sensor_trajetories[1].sensor_id);
    ASSERT_EQ(obj.position(0), track.position(0));
    ASSERT_EQ(obj.position(1), track.position(1));

}

TEST_F(FusionSystemTest, keep_one_track_by_KM_test)
{
    // 第一帧
    SensorFrame frame;
    SensorObject obj;
    frame.sensor_type = SensorType::V2X;
    frame.is_fused = 0;
    obj.time_stamp = 1;
    obj.life_time = 2;
    obj.id = 25;
    obj.position << 0, 20;
    obj.velocity << 20, 0;
    obj.acceleration << 0, 0;
    obj.pos_variance << 1, 0, 0, 1;
    obj.vel_variance << 1, 0, 0, 1;
    obj.acc_variance << 1, 0, 0, 1;
    obj.sensor_type = 1;
    frame.time_stamp = obj.time_stamp;
    frame.sensors.push_back(obj);
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);

    Fuse();
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    ASSERT_EQ(uint32_t(1), track_list.size());

    const FusionTrack& track = track_list[0];
    ASSERT_EQ(obj.id, track.sensor_trajetories[1].sensor_id);
    ASSERT_EQ(obj.position(0), track.position(0));
    ASSERT_EQ(obj.position(1), track.position(1));

    // 第二帧
    frame.sensors.clear();
    obj.time_stamp = 2;
    obj.life_time = 3;
    obj.position << 20.05, 19.95;
    obj.velocity << 20, 0;
    frame.time_stamp = obj.time_stamp;
    frame.sensors.push_back(obj);
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);

    Fuse();
    ASSERT_EQ(uint32_t(1), track_list.size());

    ASSERT_EQ(obj.id, track.sensor_trajetories[1].sensor_id);
    ASSERT_NEAR(obj.position(0), track.position(0), 0.05);
    ASSERT_NEAR(obj.position(1), track.position(1), 0.05);

}

TEST_F(FusionSystemTest, one_track_go_dead_test)
{
    // 第一帧
    SensorFrame frame;
    SensorObject obj;
    frame.sensor_type = SensorType::V2X;
    frame.is_fused = 0;
    obj.time_stamp = 1;
    obj.life_time = 2;
    obj.id = 25;
    obj.position << 0, 20;
    obj.velocity << 20, 0;
    obj.acceleration << 0, 0;
    obj.pos_variance << 1, 0, 0, 1;
    obj.vel_variance << 1, 0, 0, 1;
    obj.acc_variance << 1, 0, 0, 1;
    obj.sensor_type = 1;
    frame.time_stamp = obj.time_stamp;
    frame.sensors.push_back(obj);
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);

    Fuse();
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    ASSERT_EQ(uint32_t(1), track_list.size());

    const FusionTrack& track = track_list[0];
    ASSERT_EQ(obj.id, track.sensor_trajetories[1].sensor_id);
    ASSERT_EQ(obj.position(0), track.position(0));
    ASSERT_EQ(obj.position(1), track.position(1));

    // 第二帧
    frame.sensors.clear();
    obj.time_stamp = 2;
    obj.life_time = 3;
    obj.position << 35, 7;
    obj.velocity << 0, 0;
    frame.time_stamp = obj.time_stamp;
    frame.sensors.push_back(obj);
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);

    Fuse();
    ASSERT_EQ(uint32_t(2), track_list.size());

    frame.time_stamp++;
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);
    Fuse();
    ASSERT_EQ(uint32_t(2), track_list.size());

    frame.time_stamp++;
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);
    Fuse();
    ASSERT_EQ(uint32_t(1), track_list.size());
}

TEST_F(FusionSystemTest, one_track_with_two_sensors_test)
{
    // 第一帧 V2X
    SensorFrame frame;
    SensorObject obj;
    frame.sensor_type = SensorType::V2X;
    frame.is_fused = 0;
    obj.time_stamp = 1;
    obj.life_time = 2;
    obj.id = 25;
    obj.position << 0, 20;
    obj.velocity << 20, 0;
    obj.acceleration << 0, 0;
    obj.pos_variance << 1, 0, 0, 1;
    obj.vel_variance << 1, 0, 0, 1;
    obj.acc_variance << 1, 0, 0, 1;
    obj.sensor_type = 1;
    frame.time_stamp = obj.time_stamp;
    frame.sensors.push_back(obj);
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);

    Fuse();
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    ASSERT_EQ(uint32_t(1), track_list.size());

    const FusionTrack& track = track_list[0];
    ASSERT_EQ(obj.id, track.sensor_trajetories[1].sensor_id);
    ASSERT_EQ(obj.position(0), track.position(0));
    ASSERT_EQ(obj.position(1), track.position(1));

    // 第二帧 CAMERA
    frame.sensor_type = SensorType::CAMERA;
    frame.sensors.clear();
    obj.time_stamp = 2;
    obj.life_time = 3;
    obj.id = 2;
    obj.position << 20.05, 19.95;
    obj.velocity << 20, 0;
    obj.sensor_type = 0;
    frame.time_stamp = obj.time_stamp;
    frame.sensors.push_back(obj);
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);
    Fuse();
    ASSERT_EQ(uint32_t(1), track_list.size());
    ASSERT_EQ(3, track.fusion_status);
}

