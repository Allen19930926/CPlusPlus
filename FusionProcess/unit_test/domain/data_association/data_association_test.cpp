#include "domain/data_association/data_association.h"

#include "gtest/gtest.h"
#include "track_manager_mock.h"
#include "stub/stub.h"
#include "stub/addr_pri.h"

struct DataAssociationTest : testing::Test
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


TEST_F(TrackPreditionTest, predict_with_no_track_test)
{
    ASSERT_FALSE(preditor.Predict(0, SensorType::V2X));
}


