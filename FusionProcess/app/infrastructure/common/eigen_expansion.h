#ifndef C5B36BA6_0447_4448_B202_C2C5BF9D1EFC
#define C5B36BA6_0447_4448_B202_C2C5BF9D1EFC

#include <Eigen/Dense>

class EigenExpansion
{
public:
    /**
     * @brief 移除Eigen矩阵的指定行和列（十字移除）
     * @param matrix 待操作的矩阵
     * @param row 行
     * @param col 列
     */
    static void RemoveSpecRowAndCol(Eigen::MatrixXd& matrix, uint32_t row, uint32_t col);
};

#endif /* C5B36BA6_0447_4448_B202_C2C5BF9D1EFC */
