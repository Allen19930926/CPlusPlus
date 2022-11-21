/*********************************************************************************
 * @file        hungarian.h
 * @brief        匈牙利算法实现
 * @details
 * @author      zhoucw
 * @date        2022/10/27
 * @copyright   Copyright (c) 2022 Gohigh Fusion Division.
 ********************************************************************************/
#ifndef __HUNGARIAN_H__
#define __HUNGARIAN_H__

#include <Eigen/Dense>

 /**
  * 匈牙利算法实现
  */
class Hungarian
{
public:
    /**
     * @brief 匈牙利算法实现构造方法
     * @param const Eigen::MatrixXd & xDistMatrix
     * @return
     */
    explicit Hungarian(const Eigen::MatrixXd& xDistMatrix);

    /**
     * @brief 获取匈牙利算法最小匹配
     * @return pair<minValue, Vector> 存储了最小值以及最小匹配的列信息
     */
    std::pair<double, Eigen::VectorXi> Solve();

private:
    double ComputeAssignmentCost(const Eigen::MatrixXd& xOldDistMatrixData);
    void BuildAssignmentVector();
    void Step2A();
    void Step2B();
    void Step3();
    void Step4(const int nRow, const int nCol);
    void Step5();
private:
    int m_nRow = 0;
    int m_nCol = 0;
    double m_dNumDim = 0;

    Eigen::MatrixXd m_xDistMatrix;
    Eigen::Matrix<bool, -1, 1> m_xCoveredColumns;
    Eigen::Matrix<bool, -1, 1> m_xCoveredRows;
    Eigen::Matrix<bool, -1, -1> m_xStartMatrix;
    Eigen::Matrix<bool, -1, -1> m_xPrimeMatrix;
    Eigen::Matrix<bool, -1, -1> m_xNewStartMatrix;
    Eigen::VectorXi m_xAssignment;/* 存储匈牙利算法最终结果的列信息 */
};

#endif // !__HUNGARIAN_H__