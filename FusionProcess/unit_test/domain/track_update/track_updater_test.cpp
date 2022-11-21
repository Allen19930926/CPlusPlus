#include "domain/track_update/track_updater.h"

#include "gtest/gtest.h"
#include "track_manager_mock.h"
#include "stub/stub.h"
#include "stub/addr_pri.h"

struct TrackUpdaterTest : testing::Test
{
    void TearDown() override
    {
        mocker.ClearOperation();
    }

    TrackUpdater updater;
    TrackManagerMocker mocker;
};

ACCESS_PRIVATE_FUN(TrackUpdater, void(const SensorObject& , FusionTrack&), UpdateTrackWithMeas);
ACCESS_PRIVATE_FUN(TrackUpdater, void(const SensorObject& , FusionTrack&), UpdateTrackCommonAttr);
ACCESS_PRIVATE_FUN(TrackUpdater, void(const SensorType , FusionTrack& ), UpdateTrackWithoutMeas);

TEST_F(TrackUpdaterTest, update_assigned_track_fail_with_error_size_test)
{
    SensorFrame frame;
    std::vector<TrackMatchPair> match_res;
    updater.UpdateAssignedTracks(frame, match_res);

    // m>s m>t
    TrackMatchPair null_pair;
    match_res.push_back(null_pair);
    updater.UpdateAssignedTracks(frame, match_res);

    // m<s  m>t
    SensorObject nullobj;
    frame.sensors.push_back(nullobj);
    frame.sensors.push_back(nullobj);
    updater.UpdateAssignedTracks(frame, match_res);

    // m>s  m<t
    frame.sensors.clear();
    mocker.MockCreateTracks(2, SensorType::CAMERA);
    updater.UpdateAssignedTracks(frame, match_res);
}

TEST_F(TrackUpdaterTest, update_assigned_camera_track_success_test)
{
    SensorFrame frame;
    frame.sensor_type = SensorType::CAMERA;
    SensorObject obj;
    obj.cipv = 1;
    obj.sensor_type = static_cast<uint32_t>(frame.sensor_type);
    frame.sensors.push_back(obj);

    mocker.MockCreateTracks(1, SensorType::CAMERA);
    auto& track_list = FusionTrackManager::GetInstance().track_list;
    track_list[0].cipv = 0;
    track_list[0].track_duration = 0;
    track_list[0].sensor_trajetories[0].kf_data.exist = true;

    TrackMatchPair pair{0,0};
    std::vector<TrackMatchPair> match_res;
    match_res.push_back(pair);

    updater.UpdateAssignedTracks(frame, match_res);

    ASSERT_EQ(uint32_t(1), track_list[0].track_duration);
    ASSERT_EQ(uint32_t(1), track_list[0].cipv);
}

TEST_F(TrackUpdaterTest, update_assigned_v2x_track_success_test)
{
    SensorFrame frame;
    frame.sensor_type = SensorType::V2X;
    SensorObject obj;
    obj.cipv = 1;
    obj.sensor_type = static_cast<uint32_t>(frame.sensor_type);
    frame.sensors.push_back(obj);

    mocker.MockCreateTracks(1, SensorType::V2X);
    auto& track_list = FusionTrackManager::GetInstance().track_list;
    track_list[0].cipv = 0;
    track_list[0].track_duration = 0;
    track_list[0].sensor_trajetories[0].kf_data.exist = true;
    track_list[0].sensor_trajetories[1].kf_data.exist = true;

    TrackMatchPair pair{0,0};
    std::vector<TrackMatchPair> match_res;
    match_res.push_back(pair);

    updater.UpdateAssignedTracks(frame, match_res);

    ASSERT_EQ(uint32_t(1), track_list[0].track_duration);
    ASSERT_EQ(uint32_t(0), track_list[0].cipv);
}

TEST_F(TrackUpdaterTest, update_track_with_unknown_meas_fail_test)
{
    SensorObject meas;
    FusionTrack track;
    meas.cipv = 1;
    meas.sensor_type = 10;
    track.cipv = 0;

    call_private_fun::TrackUpdaterUpdateTrackWithMeas(updater, meas, track);

    ASSERT_NE(meas.cipv, track.cipv);
}

TEST_F(TrackUpdaterTest, update_track_common_attr_with_camera_meas_success_test)
{
    SensorObject meas;
    FusionTrack track;
    meas.cipv = 1;
    meas.sensor_type = 0;
    track.cipv = 0;

    call_private_fun::TrackUpdaterUpdateTrackCommonAttr(updater, meas, track);

    ASSERT_EQ(meas.cipv, track.cipv);
}

TEST_F(TrackUpdaterTest, update_track_common_attr_with_v2x_meas_success_test)
{
    SensorObject meas;
    FusionTrack track;
    meas.sensor_type = 1;
    meas.object_type = 0;
    track.object_type = 1;

    call_private_fun::TrackUpdaterUpdateTrackCommonAttr(updater, meas, track);

    ASSERT_EQ(meas.object_type, track.object_type);
}

TEST_F(TrackUpdaterTest, update_track_common_attr_with_frontradar_meas_success_test)
{
    SensorObject meas;
    FusionTrack track;
    track.fusion_status = 0;
    meas.sensor_type = 2;

    call_private_fun::TrackUpdaterUpdateTrackCommonAttr(updater, meas, track);

    ASSERT_NE(uint32_t(0), (track.fusion_status & uint32_t(1) << meas.sensor_type));
}

TEST_F(TrackUpdaterTest, update_unmatched_tracks_fail_with_error_size_test)
{
    std::vector<uint32_t> unmatched_idx;
    updater.UpdateUnassignedTracks(SensorType::CAMERA, unmatched_idx);

    unmatched_idx.push_back(1);
    updater.UpdateUnassignedTracks(SensorType::CAMERA, unmatched_idx);
}

TEST_F(TrackUpdaterTest, update_unmatched_tracks_success_test)
{
    std::vector<uint32_t> unmatched_idx {0};
    mocker.MockCreateTracks(1, SensorType::CAMERA);
    auto& track_list = FusionTrackManager::GetInstance().track_list;
    track_list[0].fusion_status = 1;
    track_list[0].track_duration = 7;
    track_list[0].sensor_trajetories[0].coasting_age = 2;

    updater.UpdateUnassignedTracks(SensorType::CAMERA, unmatched_idx);

    ASSERT_EQ(0, track_list[0].track_duration);
}

TEST_F(TrackUpdaterTest, update_unmatched_track_trajetory_become_dead_test)
{

    FusionTrack track;
    track.is_dead = 0;
    track.fusion_status = 1;
    track.sensor_trajetories[0].coasting_age = 2;

    call_private_fun::TrackUpdaterUpdateTrackWithoutMeas(updater, SensorType::CAMERA, track);

    ASSERT_EQ(1, track.is_dead);
}

TEST_F(TrackUpdaterTest, update_unmatched_track_trajetory_success_test)
{
    FusionTrack track;
    track.is_dead = 0;
    track.sensor_trajetories[0].coasting_age = 0;
    track.sensor_trajetories[0].kf_data.exist = true;
    track.sensor_trajetories[0].kf_data.x_prior(0) = 20;
    track.sensor_trajetories[0].kf_data.x_poster(0) = 0;
    
    call_private_fun::TrackUpdaterUpdateTrackWithoutMeas(updater, SensorType::CAMERA, track);

    ASSERT_EQ(track.sensor_trajetories[0].kf_data.x_prior(0), track.sensor_trajetories[0].kf_data.x_poster(0));
}


