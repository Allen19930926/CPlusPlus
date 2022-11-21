#include "domain/track_prediction/track_predictor.h"

#include "gtest/gtest.h"
#include "track_manager_mock.h"
#include "stub/stub.h"
#include "stub/addr_pri.h"

struct TrackPreditionTest : testing::Test
{
    void TearDown() override
    {
        mocker.ClearOperation();
    }

    TrackPredictor preditor;
    TrackManagerMocker mocker;
};

ACCESS_PRIVATE_FUN(TrackPredictor, void(const uint32_t , const float , FusionTrack& ), PredictOneTrack);
ACCESS_PRIVATE_FUN(TrackPredictor, void(const uint32_t , const float , FusionTrackKfData& ), CompensateEgoMotion);


TEST_F(TrackPreditionTest, predict_with_no_index_test)
{
    std::vector<uint32_t>  sensor_track_idx;
    ASSERT_FALSE(preditor.Predict(0, SensorType::V2X, sensor_track_idx));
}

TEST_F(TrackPreditionTest, predict_with_no_track_test)
{
    std::vector<uint32_t>  sensor_track_idx;
    sensor_track_idx.push_back(0);
    ASSERT_FALSE(preditor.Predict(0, SensorType::V2X, sensor_track_idx));
}

TEST_F(TrackPreditionTest, predict_test)
{
    std::vector<uint32_t>  sensor_track_idx {0,2,5,8};
    mocker.MockCreateTracks(10, SensorType::V2X);
    ASSERT_TRUE(preditor.Predict(0, SensorType::V2X, sensor_track_idx));
}

TEST_F(TrackPreditionTest, predict_motion_compensation_success_test)
{
    mocker.MockCreateTracks(10, SensorType::V2X);
    auto kf_data = FusionTrackManager::GetInstance().track_list[0].sensor_trajetories[1].kf_data;
    kf_data.x_prior.setOnes();
    kf_data.x_prior(0) = 20;
    float x_prior = kf_data.x_prior(0);
    call_private_fun::TrackPredictorCompensateEgoMotion(preditor, 100, 7.8, kf_data);
    float x_poster = kf_data.x_prior(0);

    ASSERT_GE(fabs(x_prior - x_poster), 1);
}

TEST_F(TrackPreditionTest, predict_motion_compensation_fail_test)
{
    mocker.MockCreateTracks(10, SensorType::V2X);
    auto kf_data = FusionTrackManager::GetInstance().track_list[0].sensor_trajetories[1].kf_data;
    kf_data.x_prior.setOnes();
    kf_data.x_prior(0) = 20;
    float x_prior = kf_data.x_prior(0);
    call_private_fun::TrackPredictorCompensateEgoMotion(preditor, 1, 0.0, kf_data);
    float x_poster = kf_data.x_prior(0);

    ASSERT_LE(fabs(x_prior - x_poster), 1E-3);
}

TEST_F(TrackPreditionTest, predict_one_track_test)
{
    mocker.MockCreateTracks(1, SensorType::V2X);
    auto& track = FusionTrackManager::GetInstance().track_list[0];
    track.time_stamp = 101;

    for (uint32_t i=0; i<2; i++)
    {
        auto& kf_data = track.sensor_trajetories[i].kf_data;
        kf_data.exist = true;
        kf_data.x_prior.setOnes();
        kf_data.x_prior(0) = 20;
    }

    track.sensor_trajetories[0].kf_data.exist = false;

    uint32_t x_camera_prior = track.sensor_trajetories[0].kf_data.x_prior(0);
    uint32_t x_v2x_prior = track.sensor_trajetories[1].kf_data.x_prior(0);
    call_private_fun::TrackPredictorPredictOneTrack(preditor, track.time_stamp + 30, 7.8, track);
    uint32_t x_camera_poster = track.sensor_trajetories[0].kf_data.x_prior(0);
    uint32_t x_v2x_poster = track.sensor_trajetories[1].kf_data.x_prior(0);

    ASSERT_NE(x_v2x_prior, x_v2x_poster);
    ASSERT_EQ(x_camera_prior, x_camera_poster);

}


