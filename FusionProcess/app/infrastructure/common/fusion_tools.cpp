#include "fusion_tools.h"
#include <chrono>

void FusionTool::RemoveSpecRowAndCol(Eigen::MatrixXd& matrix, uint32_t row, uint32_t col)
{
    if (matrix.rows() == 0 || matrix.cols() == 0)
    {
        return ;
    }

    uint32_t numRows = matrix.rows() - 1;
    uint32_t numCols = matrix.cols() - 1;

    if( col < numCols ) {
        matrix.block(0, col, numRows, numCols - col) =
        matrix.block(0, col + 1, numRows, numCols - col);
    }

    if( row < numRows ) {
        matrix.block(row,0,numRows-row,numCols) =
        matrix.block(row+1,0,numRows-row,numCols);
    }

    matrix.conservativeResize(numRows,numCols);
}

uint64_t FusionTool::GetCurrentTime()
{
    // 获取操作系统当前时间点（精确到微秒）
    std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds> tpMicro
        = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
    // (微秒精度的)时间点 => (微秒精度的)时间戳
    time_t totalMicroSeconds = tpMicro.time_since_epoch().count();

    uint64_t currentTime = ((uint64_t)totalMicroSeconds)/1000;

    return currentTime;
}
