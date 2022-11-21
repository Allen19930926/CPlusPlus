#include "hungarian.h"
#include <glog/logging.h>
#include <cfloat> // for DBL_MAX
#include <cmath>  // for fabs()

Hungarian::Hungarian(const Eigen::MatrixXd& xDistMatrix)
{
    m_xDistMatrix = xDistMatrix;

    m_nRow = xDistMatrix.rows();
    m_nCol = xDistMatrix.cols();

    m_xAssignment.resize(m_nRow);
    // 将 Assignment 结果初始化成 -1
    m_xAssignment.setConstant(-1);

    m_xCoveredColumns.resize(m_nCol);
    m_xCoveredColumns.setConstant(false);

    m_xCoveredRows.resize(m_nRow);
    m_xCoveredRows.setConstant(false);

    m_xStartMatrix.resize(m_nRow, m_nCol);
    m_xStartMatrix.setConstant(false);

    m_xPrimeMatrix.resize(m_nRow, m_nCol);
    m_xPrimeMatrix.setConstant(false);

    m_xNewStartMatrix.resize(m_nRow, m_nCol);
    m_xNewStartMatrix.setConstant(false);
}

std::pair<double, Eigen::VectorXi> Hungarian::Solve()
{
    auto xOldDistMatrixData = m_xDistMatrix;

    // 判断矩阵元素是否有负数
    for (int nRow = 0; nRow < m_nRow; ++nRow)
    {
        for (int nCol = 0; nCol < m_nCol; ++nCol)
        {
            if (m_xDistMatrix(nRow, nCol) < 0)
            {
                LOG(WARNING) << "All matrix elements have to be non-negative.";
            }
        }
    }
    double minValue = 0;

    if (m_nRow <= m_nCol)
    {
        m_dNumDim = m_nRow;

        for (int nCol = 0; nCol < m_nCol; ++nCol)
        {
            minValue = m_xDistMatrix(0, nCol);
            // 遍历 nRow 行找到最小的元素
            for (int nRow = 1; nRow < m_nCol; ++nRow)
            {
                minValue = m_xDistMatrix(nRow, nCol) < minValue ? m_xDistMatrix(nRow, nCol) : minValue;
            }
            // 遍历 nRow 行所有列，每一列均减去最小元素
            for (int nRow = 0; nRow < m_nCol; ++nRow)
            {
                m_xDistMatrix(nRow, nCol) -= minValue;
            }
        }

        // 指定第一步和 2a 步
        for (int nRow = 0; nRow < m_nRow; ++nRow)
        {
            for (int nCol = 0; nCol < m_nCol; ++nCol)
            {
                if (fabs(m_xDistMatrix(nRow, nCol)) < DBL_EPSILON)
                {
                    if (!m_xCoveredColumns(nCol))
                    {
                        m_xStartMatrix(nRow, nCol) = true;
                        m_xCoveredColumns(nCol) = true;
                        break;
                    }
                }
            }
        }
    }
    else
    {
        m_dNumDim = m_nCol;
        for (int nRow = 0; nRow < m_nRow; ++nRow)
        {
            minValue = m_xDistMatrix(nRow, 0);
            for (int nCol = 1; nCol < m_nCol; ++nCol)
            {
                minValue = m_xDistMatrix(nRow, nCol) < minValue ? m_xDistMatrix(nRow, nCol) : minValue;
            }

            for (int nCol = 0; nCol < m_nRow; ++nCol)
            {
                m_xDistMatrix(nRow, nCol) -= minValue;
            }
        }

        for (int nCol = 0; nCol < m_nCol; ++nCol)
        {
            for (int nRow = 0; nRow < m_nRow; ++nRow)
            {
                if (fabs(m_xDistMatrix(nRow, nCol)) < DBL_EPSILON)
                {
                    if (!m_xCoveredRows(nRow))
                    {
                        m_xStartMatrix(nRow, nCol) = true;
                        m_xCoveredColumns(nCol) = true;
                        m_xCoveredRows(nRow) = true;
                        break;
                    }
                }
            }
        }
        m_xCoveredRows.setConstant(false);
    }

    Step2B();
    double dResult = ComputeAssignmentCost(xOldDistMatrixData);

    return  std::make_pair(dResult, m_xAssignment);
}

void Hungarian::BuildAssignmentVector()
{
    for (int nRow = 0; nRow < m_nRow; ++nRow)
    {
        for (int nCol = 0; nCol < m_nRow; ++nCol)
        {
            if (m_xStartMatrix(nRow, nCol))
            {
#ifdef ONE_INDEXING
                m_xAssignment(nRow) = (nCol + 1);
#else
                m_xAssignment(nRow) = nCol;
#endif
                break;
            }
        }
    }
}

double Hungarian::ComputeAssignmentCost(const Eigen::MatrixXd& xOldDistMatrixData)
{
    double dResult = 0;
    for (int nRow = 0; nRow < m_nRow; ++nRow)
    {
        int nCol = m_xAssignment(nRow);
        if (nCol >= 0)
        {
            dResult += xOldDistMatrixData(nRow, nCol);
        }
    }
    return dResult;
}

void Hungarian::Step2A()
{
    for (int nRow = 0; nRow < m_nRow; ++nRow)
    {
        for (int nCol = 0; nCol < m_nCol; ++nCol)
        {
            if (m_xStartMatrix(nRow, nCol))
            {
                m_xCoveredColumns(nCol) = true;
                break;
            }
        }
    }
    Step2B();
}

void Hungarian::Step2B()
{
    int nCoveredColumns = 0;
    for (int nCol = 0; nCol < m_nCol; ++nCol)
    {
        if (m_xCoveredColumns(nCol)) { nCoveredColumns++; }
    }

    if (nCoveredColumns == m_dNumDim)
    {
        BuildAssignmentVector();
    }
    else
    {
        Step3();
    }
}

void Hungarian::Step3()
{
    bool bZeroFound = true;
    while (bZeroFound)
    {
        bZeroFound = false;

        for (int nCol = 0; nCol < m_nCol; ++nCol)
        {
            if (m_xCoveredColumns(nCol)) { continue; }

            for (int nRow = 0; nRow < m_nRow; ++nRow)
            {
                if (m_xCoveredRows(nRow) || (fabs(m_xDistMatrix(nRow, nCol)) >= DBL_EPSILON)) { continue; }

                m_xPrimeMatrix(nRow, nCol) = true;

                int nStartCol = 0;
                for (; nStartCol < m_nCol; ++nStartCol)
                {
                    if (m_xStartMatrix(nRow, nStartCol)) { break; }
                }
                if (nStartCol == m_nCol)
                {
                    Step4(nRow, nCol);
                    return;
                }

                m_xCoveredRows(nRow) = true;
                m_xCoveredColumns(nStartCol) = false;
                bZeroFound = true;
                break;
            }
        }
    }
    Step5();
}

void Hungarian::Step4(const int nRow, const int nCol)
{
    auto xNewStartMatrix = m_xStartMatrix;
    xNewStartMatrix(nRow, nCol) = true;

    int nStartCol = nCol, nStartRow = 0;
    // 遍历找到开始的列
    for (; nStartRow < m_nRow; ++nStartRow)
    {
        if (m_xStartMatrix(nStartRow, nStartCol)) { break; }
    }

    int nPrimeRow = 0, nPrimeCol = 0;
    while (nStartRow < m_nRow)
    {
        xNewStartMatrix(nStartRow, nStartCol) = false;

        nPrimeRow = nStartRow;

        for (nPrimeCol = 0; nPrimeCol < m_nCol; ++nPrimeCol)
        {
            if (m_xPrimeMatrix(nPrimeRow, nPrimeCol)) { break; }
        }

        xNewStartMatrix(nPrimeRow, nPrimeCol) = true;

        nStartCol = nPrimeCol;

        for (nStartRow = 0; nStartRow < m_nRow; ++nStartRow)
        {
            if (m_xStartMatrix(nStartRow, nStartCol)) { break; }
        }
    }

    m_xPrimeMatrix.setConstant(false);
    m_xStartMatrix = xNewStartMatrix;
    m_xCoveredRows.setConstant(false);

    Step2A();
}

void Hungarian::Step5()
{
    double dTempValue = DBL_MAX;
    for (int nRow = 0; nRow < m_nRow; ++nRow)
    {
        if (m_xCoveredRows(nRow)) { continue; }

        for (int nCol = 0; nCol < m_nCol; ++nCol)
        {
            if (m_xCoveredColumns(nCol)) { continue; }

            dTempValue = m_xDistMatrix(nRow, nCol) < dTempValue ? m_xDistMatrix(nRow, nCol) : dTempValue;
        }
    }

    for (int nRow = 0; nRow < m_nRow; ++nRow)
    {
        if (!m_xCoveredRows(nRow)) { continue; }

        for (int nCol = 0; nCol < m_nCol; ++nCol)
        {
            m_xDistMatrix(nRow, nCol) += dTempValue;
        }
    }

    for (int nCol = 0; nCol < m_nCol; ++nCol)
    {
        if (m_xCoveredColumns(nCol)) { continue; }
        for (int nRow = 0; nRow < m_nRow; ++nRow)
        {
            m_xDistMatrix(nRow, nCol) -= dTempValue;
        }
    }
    Step3();
}