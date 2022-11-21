#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "infrastructure/mahalanobis/mahalanobis.h"

// �������Ͼ����е�ŷʽ����
TEST(MahalanobisTest, test_GetEdclideanDistance)
{
    Eigen::Vector2f xPoint1;
    xPoint1 << 5, 4;

    Eigen::Vector2f xPoint2;
    xPoint2 << 3, 4;

    ASSERT_EQ(Mahalanobis::GetEdclideanDistance(xPoint1, xPoint2), 2);
}

TEST(MahalanobisTest, test_GetMahalDistance_1)
{
    Eigen::MatrixXf xData(4, 2);
    xData << 3, 4,
        5, 6,
        2, 2,
        8, 4;
    //    EXPECT_FLOAT_EQ(Mahalanobis::GetMahalDistance(xData, 0, 1), 1.24316);
}

TEST(MahalanobisTest, test_GetMahalDistance_2)
{
    Eigen::Matrix2f xCovariance;
    xCovariance << 1, 0,
        0, 1;

    Eigen::Vector2f xPoint1, xPoint2;
    xPoint1 << 5, 4;
    xPoint2 << 3, 4;

    ASSERT_EQ(Mahalanobis::GetMahalDistance(xPoint1, xPoint2, xCovariance), 2);
}
