#include "application/fusion_system.h"
#include "infrastructure/track_object/fusion_track_manager.h"
#include "infrastructure/common/ipc_data.h"

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

    FusionSystem sys;
};



TEST_F(FusionSystemTest, fuse_with_error_length_test)
{
    CDD_Fusion_ObjInfo_BUS object;
    sys.Fuse(reinterpret_cast<uint8_t*>(&object), sizeof(object));
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    ASSERT_EQ(0, track_list.size());

}

TEST_F(FusionSystemTest, fuse_with_empty_input_test)
{
    CDD_Fusion_ObjInfo_Array40 first_frame;
    memset(&first_frame, 0, sizeof(first_frame));
    sys.Fuse(reinterpret_cast<uint8_t*>(&first_frame), sizeof(first_frame));
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    ASSERT_EQ(0, track_list.size());

}
TEST_F(FusionSystemTest, create_one_track_test)
{
    CDD_Fusion_ObjInfo_Array40 first_frame;
    first_frame[0].De_Timestamp_u32 = 1;
    first_frame[0].De_life_time_u32 = 2;
    first_frame[0].De_ID_u8 = 25;
    first_frame[0].De_dx_f32 = 0;
    first_frame[0].De_dy_f32 = 20;
    first_frame[0].De_vx_f32 = 0;
    first_frame[0].De_vy_f32 = 0;
    first_frame[0].De_source_u8 = 1;

    sys.Fuse(reinterpret_cast<uint8_t*>(&first_frame), sizeof(first_frame));
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    ASSERT_EQ(1, track_list.size());

    const FusionTrack& track = track_list[0];
    ASSERT_EQ(first_frame[0].De_ID_u8, track.sensor_trajetories[1].sensor_id);
    ASSERT_EQ(first_frame[0].De_dx_f32, track.position(0));
    ASSERT_EQ(first_frame[0].De_dy_f32, track.position(1));
}

TEST_F(FusionSystemTest, keep_one_track_by_id_test)
{
    CDD_Fusion_ObjInfo_Array40 first_frame;
    first_frame[0].De_Timestamp_u32 = 1;
    first_frame[0].De_life_time_u32 = 2;
    first_frame[0].De_ID_u8 = 25;
    first_frame[0].De_dx_f32 = 0;
    first_frame[0].De_dy_f32 = 20;
    first_frame[0].De_vx_f32 = 20;
    first_frame[0].De_vy_f32 = 0;
    first_frame[0].De_ax_f32 = 0;
    first_frame[0].De_ay_f32 = 0;
    first_frame[0].De_dxVariance_f32 = 1;
    first_frame[0].De_dyVariance_f32 = 1;
    first_frame[0].De_vxVariance_f32 = 1;
    first_frame[0].De_vyVariance_f32 = 1;
    first_frame[0].De_axVariance_f32 = 1;
    first_frame[0].De_ayVariance_f32 = 1;
    first_frame[0].De_source_u8 = 1;

    sys.Fuse(reinterpret_cast<uint8_t*>(&first_frame), sizeof(first_frame));
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    ASSERT_EQ(1, track_list.size());

    const FusionTrack& track = track_list[0];
    ASSERT_EQ(first_frame[0].De_ID_u8, track.sensor_trajetories[1].sensor_id);
    ASSERT_EQ(first_frame[0].De_dx_f32, track.position(0));
    ASSERT_EQ(first_frame[0].De_dy_f32, track.position(1));


    first_frame[0].De_Timestamp_u32 = 2;
    first_frame[0].De_life_time_u32 = 3;
    first_frame[0].De_ID_u8 = 25;
    first_frame[0].De_dx_f32 = 20;
    first_frame[0].De_dy_f32 = 20;
    first_frame[0].De_vx_f32 = 20;
    first_frame[0].De_vy_f32 = 0;
    sys.Fuse(reinterpret_cast<uint8_t*>(&first_frame), sizeof(first_frame));
    ASSERT_EQ(1, track_list.size());

    ASSERT_EQ(first_frame[0].De_ID_u8, track.sensor_trajetories[1].sensor_id);
    ASSERT_EQ(first_frame[0].De_dx_f32, track.position(0));
    ASSERT_EQ(first_frame[0].De_dy_f32, track.position(1));

}

TEST_F(FusionSystemTest, keep_one_track_by_KM_test)
{
    // 第一帧
    CDD_Fusion_ObjInfo_Array40 first_frame;
    first_frame[0].De_Timestamp_u32 = 1;
    first_frame[0].De_life_time_u32 = 2;
    first_frame[0].De_ID_u8 = 25;
    first_frame[0].De_dx_f32 = 0;
    first_frame[0].De_dy_f32 = 20;
    first_frame[0].De_dy_f32 = 20;
    first_frame[0].De_vx_f32 = 20;
    first_frame[0].De_vy_f32 = 0;
    first_frame[0].De_ax_f32 = 0;
    first_frame[0].De_ay_f32 = 0;
    first_frame[0].De_dxVariance_f32 = 1;
    first_frame[0].De_dyVariance_f32 = 1;
    first_frame[0].De_vxVariance_f32 = 1;
    first_frame[0].De_vyVariance_f32 = 1;
    first_frame[0].De_axVariance_f32 = 1;
    first_frame[0].De_ayVariance_f32 = 1;
    first_frame[0].De_source_u8 = 1;

    sys.Fuse(reinterpret_cast<uint8_t*>(&first_frame), sizeof(first_frame));
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    ASSERT_EQ(1, track_list.size());

    const FusionTrack& track = track_list[0];
    ASSERT_EQ(first_frame[0].De_ID_u8, track.sensor_trajetories[1].sensor_id);
    ASSERT_EQ(first_frame[0].De_dx_f32, track.position(0));
    ASSERT_EQ(first_frame[0].De_dy_f32, track.position(1));

    // 第二帧
    first_frame[0].De_Timestamp_u32 = 2;
    first_frame[0].De_life_time_u32 = 3;
    first_frame[0].De_ID_u8 = 2;
    first_frame[0].De_dx_f32 = 20.05;
    first_frame[0].De_dy_f32 = 19.95;
    first_frame[0].De_vx_f32 = 20;
    first_frame[0].De_vy_f32 = 0;
    sys.Fuse(reinterpret_cast<uint8_t*>(&first_frame), sizeof(first_frame));
    ASSERT_EQ(1, track_list.size());

    ASSERT_EQ(first_frame[0].De_ID_u8, track.sensor_trajetories[1].sensor_id);
    ASSERT_NEAR(first_frame[0].De_dx_f32, track.position(0), 0.05);
    ASSERT_NEAR(first_frame[0].De_dy_f32, track.position(1), 0.05);

}

TEST_F(FusionSystemTest, one_track_go_dead_test)
{
    // 第一帧
    CDD_Fusion_ObjInfo_Array40 first_frame;
    first_frame[0].De_Timestamp_u32 = 1;
    first_frame[0].De_life_time_u32 = 2;
    first_frame[0].De_ID_u8 = 25;
    first_frame[0].De_dx_f32 = 0;
    first_frame[0].De_dy_f32 = 20;
    first_frame[0].De_dy_f32 = 20;
    first_frame[0].De_vx_f32 = 20;
    first_frame[0].De_vy_f32 = 0;
    first_frame[0].De_ax_f32 = 0;
    first_frame[0].De_ay_f32 = 0;
    first_frame[0].De_dxVariance_f32 = 1;
    first_frame[0].De_dyVariance_f32 = 1;
    first_frame[0].De_vxVariance_f32 = 1;
    first_frame[0].De_vyVariance_f32 = 1;
    first_frame[0].De_axVariance_f32 = 1;
    first_frame[0].De_ayVariance_f32 = 1;
    first_frame[0].De_source_u8 = 1;

    sys.Fuse(reinterpret_cast<uint8_t*>(&first_frame), sizeof(first_frame));
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    ASSERT_EQ(1, track_list.size());

    const FusionTrack& track = track_list[0];
    ASSERT_EQ(first_frame[0].De_ID_u8, track.sensor_trajetories[1].sensor_id);
    ASSERT_EQ(first_frame[0].De_dx_f32, track.position(0));
    ASSERT_EQ(first_frame[0].De_dy_f32, track.position(1));

    // 第二帧
    first_frame[0].De_Timestamp_u32 = 2;
    first_frame[0].De_life_time_u32 = 3;
    first_frame[0].De_ID_u8 = 2;
    first_frame[0].De_dx_f32 = 35;
    first_frame[0].De_dy_f32 = 7;
    first_frame[0].De_vx_f32 = 0;
    first_frame[0].De_vy_f32 = 0;
    sys.Fuse(reinterpret_cast<uint8_t*>(&first_frame), sizeof(first_frame));
    ASSERT_EQ(2, track_list.size());

    first_frame[0].De_Timestamp_u32++;
    sys.Fuse(reinterpret_cast<uint8_t*>(&first_frame), sizeof(first_frame));
    ASSERT_EQ(2, track_list.size());
    first_frame[0].De_Timestamp_u32++;
    sys.Fuse(reinterpret_cast<uint8_t*>(&first_frame), sizeof(first_frame));
    ASSERT_EQ(1, track_list.size());
}

TEST_F(FusionSystemTest, one_track_with_two_sensors_test)
{
    // 第一帧 V2X
    CDD_Fusion_ObjInfo_Array40 first_frame;
    first_frame[0].De_Timestamp_u32 = 1;
    first_frame[0].De_life_time_u32 = 2;
    first_frame[0].De_ID_u8 = 25;
    first_frame[0].De_dx_f32 = 0;
    first_frame[0].De_dy_f32 = 20;
    first_frame[0].De_dy_f32 = 20;
    first_frame[0].De_vx_f32 = 20;
    first_frame[0].De_vy_f32 = 0;
    first_frame[0].De_ax_f32 = 0;
    first_frame[0].De_ay_f32 = 0;
    first_frame[0].De_dxVariance_f32 = 1;
    first_frame[0].De_dyVariance_f32 = 1;
    first_frame[0].De_vxVariance_f32 = 1;
    first_frame[0].De_vyVariance_f32 = 1;
    first_frame[0].De_axVariance_f32 = 1;
    first_frame[0].De_ayVariance_f32 = 1;
    first_frame[0].De_source_u8 = 1;

    sys.Fuse(reinterpret_cast<uint8_t*>(&first_frame), sizeof(first_frame));
    const std::vector<FusionTrack>& track_list = FusionTrackManager::GetInstance().track_list;
    ASSERT_EQ(1, track_list.size());

    const FusionTrack& track = track_list[0];
    ASSERT_EQ(first_frame[0].De_ID_u8, track.sensor_trajetories[1].sensor_id);
    ASSERT_EQ(first_frame[0].De_dx_f32, track.position(0));
    ASSERT_EQ(first_frame[0].De_dy_f32, track.position(1));

    // 第二帧 CAMERA
    first_frame[0].De_Timestamp_u32 = 2;
    first_frame[0].De_life_time_u32++;
    first_frame[0].De_ID_u8 = 2;
    first_frame[0].De_dx_f32 = 20.05;
    first_frame[0].De_dy_f32 = 19.95;
    first_frame[0].De_vx_f32 = 20;
    first_frame[0].De_vy_f32 = 0;
    first_frame[0].De_source_u8 = 0;
    sys.Fuse(reinterpret_cast<uint8_t*>(&first_frame), sizeof(first_frame));
    ASSERT_EQ(1, track_list.size());
    ASSERT_EQ(3, track.fusion_status);
}

