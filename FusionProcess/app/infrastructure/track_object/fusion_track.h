#ifndef A1368429_4ADE_4930_BDEB_451B238EB8EC
#define A1368429_4ADE_4930_BDEB_451B238EB8EC

#include "Eigen/Dense"
#include "infrastructure/sensor_object/sensor_object.h"
#include <array>

struct SensorTrajetory;

using KfVector = Eigen::Matrix<float, 6, 1>;
using KfMatrix = Eigen::Matrix<float, 6, 6>;
using SensorTrajArray = std::array<SensorTrajetory, static_cast<std::size_t>(SensorType::MAX)>;

struct alignas(16) FusionTrackKfData
{
    bool     exist;
    KfVector x_prior;
    KfMatrix p_prior;
    KfVector x_poster;
    KfMatrix p_poster;
};

struct alignas(16) SensorTrajetory
{
    uint8_t             sensor_id;
    uint8_t             coasting_age;
    FusionTrackKfData   kf_data;
};


struct alignas(16) FusionTrack
{
    uint32_t            track_id;               /* 跟踪器ID */
    uint32_t            time_stamp;             /* 时间戳 */
    uint8_t             fusion_status;
    uint8_t             is_dead;
    uint8_t             track_duration;
    uint8_t             object_type;           /* 目标类别，如车辆、行人等 */
    uint8_t             cipv;

    float               yaw;
    float               yaw_rate;
    Eigen::Vector3f     size;                  /* 目标的长宽高，unit：m */
    Eigen::Vector2f     position;              /* 目标在车辆坐标系下的X,Y,Z坐标，unit：m */
    Eigen::Matrix2f     pos_variance;          /* 目标的坐标方差  */
    Eigen::Vector2f     velocity;              /* 目标在车辆坐标系下的X,Y,Z速度，unit：m/s */
    Eigen::Matrix2f     vel_variance;          /* 目标的速度方差  */
    Eigen::Vector2f     acceleration;          /* 目标在车辆坐标系下的X,Y,Z加速度，unit：m/s^2 */
    Eigen::Matrix2f     acc_variance;          /* 目标的加速度方差  */

    SensorTrajArray     sensor_trajetories;

};

struct TrackMatchPair
{
    uint32_t object_idx;
    uint32_t track_idx;
};

#endif /* A1368429_4ADE_4930_BDEB_451B238EB8EC */
