#include "kalman_filter.h"
#include "sensor_object.h"
#include "fusion_track.h"

KalmanFilter::KalmanFilter()
{
    A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
}