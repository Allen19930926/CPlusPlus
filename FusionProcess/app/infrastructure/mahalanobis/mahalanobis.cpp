#include "mahalanobis.h"
#include "infrastructure/track_object/fusion_track.h"
#include <assert.h>

double Mahalanobis::GetMahalDistance(const Eigen::MatrixXf& xData, const int nRow1, const int nRow2)
{
    // assert((nRow1 >= 0 && nRow1 < xData.rows()) && (nRow2 >= 0 && nRow2 < xData.rows()));

    // // 获取指定两行数据的差值
    // Eigen::VectorXf xSubtemp = xData.row(nRow1) - xData.row(nRow2);

    // // 分别求每列的平均值
    // Eigen::MatrixXf xMeanData = xData.colwise().mean();

    // // 将矩阵转为向量指针
    // Eigen::RowVectorXf xMeanVecRowData(Eigen::RowVectorXf::Map(xMeanData.data(), xData.cols()));

    // // 源数据矩阵减去均值
    // Eigen::MatrixXf zeroMeanMat = xData.rowwise() - xMeanVecRowData;

    // int nEleSize = xData.rows() == 1 ? 1 : xData.rows() - 1;
    // // 计算协方差矩阵
    // Eigen::MatrixXf xCovariance = (zeroMeanMat.adjoint() * zeroMeanMat) / (double)nEleSize;

    // // 计算马氏距离
    // Eigen::MatrixXf xResult = xSubtemp.transpose() * xCovariance.inverse() * xSubtemp;

    // return sqrt(xResult(0, 0));
    return 0;
}

double Mahalanobis::GetMahalDistance(const Eigen::VectorXf& xPoint1, const Eigen::VectorXf& xPoint2, const Eigen::MatrixXf& xCovariance)
{
    // LCOV_EXCL_START
    assert(xPoint1.size() == xPoint2.size());
    // LCOV_EXCL_STOP

    Eigen::VectorXf xSubTemp = xPoint1 - xPoint2;
    Eigen::MatrixXf xResult = xSubTemp.transpose() * xCovariance.inverse() * xSubTemp;

    return sqrt(xResult(0, 0));
}

double Mahalanobis::GetEdclideanDistance(const Eigen::VectorXf& xPoint1, const Eigen::VectorXf& xPoint2)
{
    // LCOV_EXCL_START
    assert(xPoint1.rows() == xPoint2.rows());
    // LCOV_EXCL_STOP

    // 使用单位矩阵作为协方差矩阵
    Eigen::MatrixXf xCovariance(xPoint1.rows(), xPoint1.rows());
    xCovariance.setIdentity();

    return GetMahalDistance(xPoint1, xPoint2, xCovariance);
}

Eigen::MatrixXd DistCalcInterface::GetTrackTrackMahalDistance(const std::vector<FusionTrack> track_list, const std::vector<uint32_t> row_track_idx, const std::vector<uint32_t> col_track_idx)
{
    Eigen::MatrixXd cost_matrix = Eigen::MatrixXd::Constant(row_track_idx.size(), col_track_idx.size(), 0xFFFFFFFF);
    for (uint16_t i=0; i<row_track_idx.size(); i++)
    {
        const FusionTrack& row_track = track_list[row_track_idx[i]];
        KfVector row_track_state_vec;
        row_track_state_vec << row_track.position, row_track.velocity, row_track.acceleration;
        for (uint16_t j=0; j<col_track_idx.size(); j++)
        {
            const FusionTrack& col_track = track_list[col_track_idx[j]];
            KfVector col_track_state_vec;
            col_track_state_vec << col_track.position, col_track.velocity, col_track.acceleration;
            double edcl_dist = Mahalanobis::GetEdclideanDistance(row_track_state_vec, col_track_state_vec);
            if (edcl_dist >= 10)
            {
                continue;
            }
            KfVector co_variance_vec;
            co_variance_vec <<  row_track.pos_variance(0,0),  row_track.pos_variance(1,1),
                                row_track.vel_variance(0,0),  row_track.vel_variance(1,1),
                                row_track.acc_variance(0,0),  row_track.vel_variance(1,1);
            KfMatrix co_variance = co_variance_vec.asDiagonal();
            cost_matrix(i, j) = Mahalanobis::GetMahalDistance(row_track_state_vec, col_track_state_vec, co_variance);
        }
    }
    return cost_matrix;
}
