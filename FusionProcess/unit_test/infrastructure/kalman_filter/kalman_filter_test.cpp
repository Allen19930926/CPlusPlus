
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "infrastructure/kalman_filter/kalman_filter.h"
#include "infrastructure/track_object/fusion_track.h"
#include "infrastructure/sensor_object/sensor_object.h"

class KalmanFilterTest : public testing::Test
{
protected:
    static void SetUpTestCase()
    {
        pKalmanFilter = new KalmanFilter();
    }

    static void TearDownTestCase()
    {
        delete pKalmanFilter;
        pKalmanFilter = nullptr;
    }

    void SetUp() override
    {

    }

    void TearDown() override
    {
    }

protected:
    FusionTrack xFunsionTrack;

    static KalmanFilter* pKalmanFilter;
};

KalmanFilter* KalmanFilterTest::pKalmanFilter = nullptr;

// 测试卡尔曼滤波的 Predict 函数
TEST_F(KalmanFilterTest, test_kalmane_filter_predict)
{
    auto& xData = xFunsionTrack.sensor_trajetories[0].kf_data;
    xData.x_poster << 1, 1, 0, 0, 0, 0;
    xData.p_poster.setIdentity();

    pKalmanFilter->Predict(10, xData);

    KfMatrix xPResult;
    xPResult << 2602, 0, 510, 0, 50, 0,
        0, 2602, 0, 510, 0, 50,
        510, 0, 102, 0, 10, 0,
        0, 510, 0, 102, 0, 10,
        50, 0, 10, 0, 2, 0,
        0, 50, 0, 10, 0, 2;

    KfVector xXResult;
    xXResult << 1, 1, 0, 0, 0, 0;

    ASSERT_EQ(xData.p_prior, xPResult);
    ASSERT_EQ(xData.x_prior, xXResult);
}


TEST_F(KalmanFilterTest, test_kalman_filter_update)
{
    SensorObject xSensorObject;

    xSensorObject.position << 1, 2;
    xSensorObject.velocity << 3, 4;
    xSensorObject.acceleration << 5, 6;

    xSensorObject.acc_variance << 1, 0, 0, 1;
    xSensorObject.pos_variance << 1, 0, 0, 1;
    xSensorObject.vel_variance << 1, 0, 0, 1;

    KfMatrix xPResult;
    xPResult << 0.5, 0, 0, 0, 0, 0,
        0, 0.5, 0, 0, 0, 0,
        0, 0, 0.5, 0, 0, 0,
        0, 0, 0, 0.5, 0, 0,
        0, 0, 0, 0, 0.5, 0,
        0, 0, 0, 0, 0, 0.5;

    KfVector xXResult;
    xXResult << 1, 1.5, 1.5, 2, 2.5, 3;

    auto& xData = xFunsionTrack.sensor_trajetories[0].kf_data;
    xData.x_prior << 1, 1, 0, 0, 0, 0;
    xData.p_prior.setIdentity();


    pKalmanFilter->Update(xSensorObject, xData);

    ASSERT_EQ(xData.p_poster, xPResult);
    ASSERT_EQ(xData.x_poster, xXResult);
}

// 测试重载的无测量值的更新函数
TEST_F(KalmanFilterTest, test_kalman_filter_update_1)
{
    for (auto eType = (int)SensorType::CAMERA; eType < (int)SensorType::MAX; ++eType)
    {
        auto& xData = xFunsionTrack.sensor_trajetories[eType].kf_data;
        xData.x_prior << 1, 1, 0, 0, 0, 0;
        xData.p_prior.setIdentity();

        pKalmanFilter->Update(xData);

        ASSERT_EQ(xData.p_poster, xData.p_prior);
        ASSERT_EQ(xData.x_poster, xData.x_prior);
    }
}
