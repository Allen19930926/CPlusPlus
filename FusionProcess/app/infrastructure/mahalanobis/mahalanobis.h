/*********************************************************************************
 * @file        mahalanobis.h
 * @brief        马氏距离实现
 * @details            1.重载了两种实现马氏距离的方式
 *                  2.实现了计算两个向量的欧式距离方法
 * @author      zhoucw
 * @date        2022/10/27
 * @copyright   Copyright (c) 2022 Gohigh Fusion Division.
 ********************************************************************************/
#ifndef __MAHALANOBIS_H__
#define __MAHALANOBIS_H__

#include <Eigen/Dense>
#include <vector>

class FusionTrack;

class Mahalanobis {
public:
    /**
     * @brief 获取众多向量中两个向量的马氏距离
     * @param const Eigen::MatrixXf & xData 所有的向量集合
     * @param const int nRow1 向量A所在的行号
     * @param const int nRow2 向量B所在的行号
     * @return nRow1 行和 nRow2 行的马氏距离
     */
    static double GetMahalDistance(const Eigen::MatrixXf& xData, const int nRow1, const int nRow2);

    /**
     * @brief 获取两个向量的马氏距离
     * @param const Eigen::VectorXf & xPoint1 向量A
     * @param const Eigen::VectorXf & xPoint2 向量B
     * @param const Eigen::MatrixXf & xCovariance 向量间的协方差矩阵
     * @return 两个向量的马氏距离
     */
    static double GetMahalDistance(const Eigen::VectorXf& xPoint1, const Eigen::VectorXf& xPoint2, const Eigen::MatrixXf& xCovariance);

    /**
     * @brief 获取两个向量的欧式距离
     * @param const Eigen::VectorXf & xPoint1 向量A
     * @param const Eigen::VectorXf & xPoint2 向量B
     * @return 两个向量的欧式距离
     */
    static double GetEdclideanDistance(const Eigen::VectorXf& xPoint1, const Eigen::VectorXf& xPoint2);
};

class DistCalcInterface
{
public:
    /// @brief 计算track和track间的马氏距离
    /// @param track_list 所有track的列表
    /// @param row_track_idx 作为行向量的track索引列表
    /// @param col_track_idx 作为列向量的track索引列表
    /// @return 返回值N阶马氏距离方阵， n为row_track_idx， m为col_track_idx容量， N=max(n,m), 未计算部分取float_max
    static Eigen::MatrixXd GetTrackTrackMahalDistance(const std::vector<FusionTrack> track_list, const std::vector<uint32_t> row_track_idx, const std::vector<uint32_t> col_track_idx);
};
#endif // !__MAHALANOBIS_H__
