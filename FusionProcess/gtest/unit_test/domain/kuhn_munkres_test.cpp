#include "gtest/gtest.h"
#include "kuhn_munkres_algo.h"
#include <chrono>
#include "glog/logging.h"

using namespace std::chrono;
using std::cout;
using std::endl;

struct KMAlgoTest : testing::Test
{
    KMAlgorithm kmsolver;
};


TEST_F(KMAlgoTest, normal_function_test)
{
    vector<vector<double>> test_matrix = {{1, 2, 3},
                                          {2, 4, 6},
                                          {3, 6, 9}};
    vector<int> assignment;

    system_clock::time_point time_before = system_clock::now(); // 获取当前时间点
    int cost = kmsolver.Solve(test_matrix, assignment);
    system_clock::time_point time_after = system_clock::now(); // 获取当前时间点

    LOG(INFO) << "process use:" << std::chrono::duration_cast<std::chrono::microseconds>(time_after - time_before).count() << " us" << std::endl;

    LOG(INFO) << "total cost: " << cost << " match pairs: 1-" << assignment[0] << " 2-" << assignment[1] << " 3-"<< assignment[2] << endl;

    ASSERT_EQ(cost, 10);
    ASSERT_EQ(assignment[2], 0);
    ASSERT_EQ(assignment[1], 1);
    ASSERT_EQ(assignment[0], 2);
}


TEST_F(KMAlgoTest, KM_step_unit_test)
{
    
}
