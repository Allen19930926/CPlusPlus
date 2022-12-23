#include "gtest/gtest.h"
#include <glog/logging.h>
#include <algorithm>

#include "track_manager_mock.h"
#include "stub/stub.h"
#include "stub/addr_pri.h"

struct TrackManagerTest : testing::Test
{
    TrackManagerTest() : track_manager(FusionTrackManager::GetInstance()) {}
    void SetUp()
    {
    }

    void TearDown()
    {
        mocker.ClearOperation();
    }

   TrackManagerMocker mocker;
   FusionTrackManager& track_manager;
};

ACCESS_PRIVATE_FUN(FusionTrackManager, void(const SensorType , std::vector<uint32_t>& , std::vector<uint32_t>& ),   GetIntegratableTracksBySensor);
ACCESS_PRIVATE_FUN(FusionTrackManager, void(const uint32_t , const uint32_t , const SensorType), FuseOneSameTrack);
ACCESS_PRIVATE_FUN(FusionTrackManager, void(const SensorType), FuseSameTracks);

TEST_F(TrackManagerTest, create_new_track_test)
{
    mocker.MockCreateTracks(100, SensorType::CAMERA);
    ASSERT_EQ(uint32_t(50), track_manager.track_list.size());
}

TEST_F(TrackManagerTest, remove_dead_track_test)
{
    mocker.MockCreateTracks(100, SensorType::CAMERA);
    track_manager.track_list[0].is_dead = 1;
    mocker.MockRemoveTracks();
    ASSERT_EQ(uint32_t(49), track_manager.track_list.size());
}

TEST_F(TrackManagerTest, get_track_by_sensor_type_test)
{
    mocker.MockCreateTracks(10, SensorType::CAMERA);
    mocker.MockCreateTracks(15, SensorType::V2X);

    std::vector<uint32_t> v2x_idx = track_manager.GetFusionTracksOfSensor(SensorType::V2X);
    std::vector<uint32_t> camera_idx = track_manager.GetFusionTracksOfSensor(SensorType::CAMERA);

    ASSERT_EQ(uint32_t(15), v2x_idx.size());
    ASSERT_EQ(uint32_t(10), camera_idx.size());
}

TEST_F(TrackManagerTest, get_single_sensor_tracks_test)
{
    mocker.MockCreateTracks(1, SensorType::CAMERA);
    mocker.MockCreateTracks(1, SensorType::V2X);
    
    std::vector<uint32_t> camera_track_idx;
    std::vector<uint32_t> v2x_track_idx;
    call_private_fun::FusionTrackManagerGetIntegratableTracksBySensor(track_manager, SensorType::CAMERA, camera_track_idx, v2x_track_idx);

    ASSERT_EQ(uint32_t(1), v2x_track_idx.size());
    ASSERT_EQ(uint32_t(1), camera_track_idx.size());
    ASSERT_EQ(track_manager.track_list[camera_track_idx[0]].fusion_status, 1<<0);
    ASSERT_EQ(track_manager.track_list[v2x_track_idx[0]].fusion_status, 1<<1);
}

TEST_F(TrackManagerTest, fuse_single_camera_track_test)
{
    mocker.MockCreateTracks(1, SensorType::CAMERA);
    mocker.MockCreateTracks(1, SensorType::V2X);

    ASSERT_EQ(track_manager.track_list[0].fusion_status, 1);
    ASSERT_EQ(track_manager.track_list[1].fusion_status, 2);

    call_private_fun::FusionTrackManagerFuseOneSameTrack(track_manager, 1, 0, SensorType::CAMERA);

    ASSERT_EQ(track_manager.track_list[0].fusion_status, 1);
    ASSERT_EQ(track_manager.track_list[1].fusion_status, 3);
}

TEST_F(TrackManagerTest, fuse_single_v2x_track_test)
{
    mocker.MockCreateTracks(1, SensorType::CAMERA);
    mocker.MockCreateTracks(1, SensorType::V2X);

    ASSERT_EQ(track_manager.track_list[0].fusion_status, 1);
    ASSERT_EQ(track_manager.track_list[1].fusion_status, 2);

    call_private_fun::FusionTrackManagerFuseOneSameTrack(track_manager, 0, 1, SensorType::V2X);

    ASSERT_EQ(track_manager.track_list[0].fusion_status, 3);
    ASSERT_EQ(track_manager.track_list[1].fusion_status, 2);
}

TEST_F(TrackManagerTest, fuse_single_othertype_track_test)
{
    mocker.MockCreateTracks(1, SensorType::CAMERA);
    mocker.MockCreateTracks(1, SensorType::FRONT_RADAR);

    ASSERT_EQ(track_manager.track_list[0].fusion_status, 1);
    ASSERT_EQ(track_manager.track_list[1].fusion_status, 4);

    call_private_fun::FusionTrackManagerFuseOneSameTrack(track_manager, 0, 1, SensorType::FRONT_RADAR);

    ASSERT_EQ(track_manager.track_list[0].fusion_status, 5);
    ASSERT_EQ(track_manager.track_list[1].fusion_status, 4);
}

TEST_F(TrackManagerTest, fuse_track_fail_with_error_idx_test)
{
    mocker.MockCreateTracks(1, SensorType::CAMERA);
    mocker.MockCreateTracks(1, SensorType::FRONT_RADAR);

    ASSERT_EQ(track_manager.track_list[0].fusion_status, 1);
    ASSERT_EQ(track_manager.track_list[1].fusion_status, 4);

    call_private_fun::FusionTrackManagerFuseOneSameTrack(track_manager, 5, 1, SensorType::FRONT_RADAR);
    call_private_fun::FusionTrackManagerFuseOneSameTrack(track_manager, 0, 5, SensorType::FRONT_RADAR);

    ASSERT_EQ(track_manager.track_list[0].fusion_status, 1);
    ASSERT_EQ(track_manager.track_list[1].fusion_status, 4);
}

TEST_F(TrackManagerTest, fuse_tracks_fail_with_large_distance_test)
{
    mocker.MockCreateTracks(1, SensorType::CAMERA);
    mocker.MockCreateTracks(1, SensorType::FRONT_RADAR);

    ASSERT_EQ(track_manager.track_list[0].fusion_status, 1);
    ASSERT_EQ(track_manager.track_list[1].fusion_status, 4);

    track_manager.track_list[0].position << 100,100;
    track_manager.track_list[0].velocity << 0,0;
    track_manager.track_list[0].acceleration << 0,0;
    track_manager.track_list[0].pos_variance << 1, 0, 0, 1;
    track_manager.track_list[0].vel_variance << 1, 0, 0, 1;
    track_manager.track_list[0].acc_variance << 1, 0, 0, 1;
    
    track_manager.track_list[1].position << 20,20;
    track_manager.track_list[1].velocity << 0,0;
    track_manager.track_list[1].acceleration << 0,0;
    track_manager.track_list[1].pos_variance << 1, 0, 0, 1;
    track_manager.track_list[1].vel_variance << 1, 0, 0, 1;
    track_manager.track_list[1].acc_variance << 1, 0, 0, 1;

    call_private_fun::FusionTrackManagerFuseSameTracks(track_manager, SensorType::FRONT_RADAR);

    ASSERT_EQ(track_manager.track_list[0].fusion_status, 1);
    ASSERT_EQ(track_manager.track_list[1].fusion_status, 4);
}

