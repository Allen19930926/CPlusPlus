#include "eigen_expansion.h"

void EigenExpansion::RemoveSpecRowAndCol(Eigen::MatrixXf& matrix, uint32_t row, uint32_t col)
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
