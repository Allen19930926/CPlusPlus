/*********************************************************************************
 * @file        kalman_filter.h
 * @brief        卡尔曼滤波实现
 * @details
 * @author      zhoucw
 * @date        2022/10/27
 * @copyright   Copyright (c) 2022 Gohigh Fusion Division.
 ********************************************************************************/
#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include <Eigen/Dense>

#include "infrastructure/track_object/fusion_track.h"
#include "infrastructure/sensor_object/sensor_object.h"

class KalmanFilter
{
public:
    KalmanFilter();

    ~KalmanFilter() {}

	/**
	 * @brief 卡尔曼滤波预测
	 * @param const uint32_t nTimeGap 时间间隔
	 * @param FusionTrackKfData & xTrackerData 融合对象
	 * @return
	 */
	void Predict(const uint32_t nTimeGap, FusionTrackKfData& xTrackerData);

	/**
	 * @brief 卡尔曼滤波更新（带测量值）
	 * @param const SensorObject & xMeasureObj 测量对象
	 * @param FusionTrackKfData & xTrackerData 融合对象
	 * @return
	 */
	void Update(const SensorObject& xMeasureObj, FusionTrackKfData& xTrackerData);

	/**
	 * @brief 卡尔曼滤波更新（不带测量值）
	 * @param FusionTrackKfData & xTrackerData 融合对象
	 * @return
	 */
	void Update(FusionTrackKfData& xTrackerData);

private:
    // 卡尔曼滤波预测涉及的参数
    KfMatrix B;
    KfMatrix Q;
    KfVector u;

    // 卡尔曼滤波更新涉及的参数
    KfMatrix H;      // 观测方程的系数
    KfMatrix H_T;    // 观测方程系数的转置
};


#endif // !__KALMAN_FILTER_H__