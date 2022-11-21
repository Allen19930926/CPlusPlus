#include "mahalanobis.h"
#include <assert.h>
#include <glog/logging.h>

double Mahalanobis::GetMahalDistance(const Eigen::MatrixXf& xData, const int nRow1, const int nRow2)
{
    assert((nRow1 >= 0 && nRow1 < xData.rows()) && (nRow2 >= 0 && nRow2 < xData.rows()));

    // 获取指定两行数据的差值
    Eigen::VectorXf xSubtemp = xData.row(nRow1) - xData.row(nRow2);

    // 分别求每列的平均值
    Eigen::MatrixXf xMeanData = xData.colwise().mean();

    // 将矩阵转为向量指针
    Eigen::RowVectorXf xMeanVecRowData(Eigen::RowVectorXf::Map(xMeanData.data(), xData.cols()));

    // 源数据矩阵减去均值
    Eigen::MatrixXf zeroMeanMat = xData.rowwise() - xMeanVecRowData;

    int nEleSize = xData.rows() == 1 ? 1 : xData.rows() - 1;
    // 计算协方差矩阵
    Eigen::MatrixXf xCovariance = (zeroMeanMat.adjoint() * zeroMeanMat) / (double)nEleSize;

    // 计算马氏距离
    Eigen::MatrixXf xResult = xSubtemp.transpose() * xCovariance.inverse() * xSubtemp;

    return sqrt(xResult(0, 0));
}

double Mahalanobis::GetMahalDistance(const Eigen::VectorXf& xPoint1, const Eigen::VectorXf& xPoint2, const Eigen::MatrixXf& xCovariance)
{
    assert(xPoint1.size() == xPoint2.size());

    Eigen::VectorXf xSubTemp = xPoint1 - xPoint2;
    Eigen::MatrixXf xResult = xSubTemp.transpose() * xCovariance.inverse() * xSubTemp;

    return sqrt(xResult(0, 0));
}

double Mahalanobis::GetEdclideanDistance(const Eigen::VectorXf& xPoint1, const Eigen::VectorXf& xPoint2)
{
    assert(xPoint1.rows() == xPoint2.rows());

    // 使用单位矩阵作为协方差矩阵
    Eigen::MatrixXf xCovariance(xPoint1.rows(), xPoint1.rows());
    xCovariance.setIdentity();

    return GetMahalDistance(xPoint1, xPoint2, xCovariance);
}