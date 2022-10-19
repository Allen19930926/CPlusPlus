#ifndef E0CCCD54_BE0C_4E57_A95E_D2431DC67201
#define E0CCCD54_BE0C_4E57_A95E_D2431DC67201

#include <Eigen/Dense>

class SensorObject;
class FusionTrack;

class KalmanFilter
{
public:
    KalmanFilter();
    void Predict(const uint32_t time_diff, FusionTrack& track);
    void UpdateWithMeas(const uint32_t time_diff, const SensorObject& meas, FusionTrack& track);
    void UpdateWithoutMeas(const uint32_t time_diff, FusionTrack& track);
private:
    Eigen::Matrix3d A;
};

#endif /* E0CCCD54_BE0C_4E57_A95E_D2431DC67201 */
