#include "kalman_filter.h"

KalmanFilter::KalmanFilter()
{
    // 设置Q
    Q.setIdentity() * 0.001;

    // 控制变量B和u设置成0
    B.setZero();
    u.setZero();

    // 设置观测矩阵H(根据HM的推荐设置成单位矩阵)
    H.setIdentity() * 0.99999999f;
    H_T = H.transpose();
}

void KalmanFilter::Predict(const uint32_t nTimeGap, FusionTrackKfData& xTrackerData)
{
    auto funcGenerateMatrixA = [nTimeGap]() -> KfMatrix
    {
        // 先将A初始化成单位矩阵
        KfMatrix A;
        A.setIdentity();

        float xHalfSquareDT = 0.5f * nTimeGap * nTimeGap;

        A(0, 2) = nTimeGap;
        A(1, 3) = nTimeGap;
        A(2, 4) = nTimeGap;
        A(3, 5) = nTimeGap;
        A(0, 4) = xHalfSquareDT;
        A(1, 5) = xHalfSquareDT;

        return A;
    };

	KfMatrix A = funcGenerateMatrixA();

	xTrackerData.x_prior = A * xTrackerData.x_poster + B * u;
	xTrackerData.p_prior = A * xTrackerData.p_poster * A.transpose() + Q;

}

void KalmanFilter::Update(const SensorObject& xMeasureObj, FusionTrackKfData& xTrackerData)
{
	auto funcGenerateMatrixZ = [&xMeasureObj]() -> KfVector
	{
		KfVector z;
		z << xMeasureObj.position(0, 0),
			xMeasureObj.position(1, 0),
			xMeasureObj.velocity(0, 0),
			xMeasureObj.velocity(1, 0),
			xMeasureObj.acceleration(0, 0),
			xMeasureObj.acceleration(1, 0);

		return z;
	};

	// 噪声使用的是测量矩阵的方差
	auto funcGenerateMatrixR = [&xMeasureObj]()->KfMatrix
	{
		KfMatrix R;
		R.setZero();

		R(0, 0) = xMeasureObj.pos_variance(0, 0);
		R(1, 1) = xMeasureObj.pos_variance(1, 1);
		R(2, 2) = xMeasureObj.vel_variance(0, 0);
		R(3, 3) = xMeasureObj.vel_variance(1, 1);
		R(4, 4) = xMeasureObj.acc_variance(0, 0);
		R(5, 5) = xMeasureObj.acc_variance(1, 1);

		return R;
	};

	// 计算增益矩阵
	KfMatrix K = xTrackerData.p_prior * H_T * (H * xTrackerData.p_prior * H_T + funcGenerateMatrixR()).inverse();

	// 状态更新
	xTrackerData.x_poster = xTrackerData.x_prior + K * (funcGenerateMatrixZ() - H * xTrackerData.x_prior);

	// 误差协方差矩阵更新
	xTrackerData.p_poster = xTrackerData.p_prior - K * H * xTrackerData.p_prior;

}

void KalmanFilter::Update(FusionTrackKfData& xTrackerData)
{
	xTrackerData.x_poster = xTrackerData.x_prior;
	xTrackerData.p_poster = xTrackerData.p_prior;
}