
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <stub/stub.h>
#include <stub/addr_pri.h>

#include "infrastructure/hungarian/hungarian.h"

class HungarianTest : public testing::Test
{
protected:
    static void SetUpTestCase()
    {
        Eigen::Matrix3d xDistMatrix;
        xDistMatrix << 1, 2, 3,
            2, 4, 6,
            3, 6, 9;
        pHungarian = new Hungarian(xDistMatrix);
    }

    static void TearDownTestCase()
    {
        delete pHungarian;
    }

protected:
    static    Hungarian* pHungarian;

};

Hungarian* HungarianTest::pHungarian = nullptr;

TEST_F(HungarianTest, test_Solve)
{
    auto pairResult = pHungarian->Solve();

    ASSERT_EQ(pairResult.first, 10);
    ASSERT_EQ(pairResult.second(0), 2);
    ASSERT_EQ(pairResult.second(1), 1);
    ASSERT_EQ(pairResult.second(2), 0);
}