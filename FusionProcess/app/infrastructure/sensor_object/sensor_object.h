#ifndef D771E8BE_51D2_49D9_B7E0_BF7C56E6C696
#define D771E8BE_51D2_49D9_B7E0_BF7C56E6C696
#include <cstdint>
#include <Eigen/Dense>
#include <vector>

enum class SensorType : uint8_t
{
    CAMERA = 0,
    V2X,
    FRONT_RADAR,
    MAX
};

struct alignas(16) SensorObject
{
    uint32_t            time_stamp;            /* 时间戳 */
    uint8_t             id;                    /* 观测对象ID，如cameraid、radarid等 */
    float               conf;                  /*  */
    uint32_t            life_time;             /*  */
    float               yaw;                   /* 目标的方位角(朝向)，方向和vcs坐标系一致，左正右负，即逆时针正，顺时针负，unit: rad */
    float               conf_yaw;              /* 朝向角置信度[0~100] */
    float               yaw_rate;              /* 目标车辆在vcs坐标系下的角速度，unit：rad/s */
    float               acc;                   /* 目标在其行驶方向上的对地加速度大小，unit: m/s^2 */
    Eigen::Vector3f     size;                  /* 目标的长宽高，unit：m */ 
    Eigen::Vector2f     position;              /* 目标在车辆坐标系下的X,Y坐标，unit：m */ 
    Eigen::Matrix2f     pos_variance;          /* 目标的坐标方差  */
    Eigen::Vector2f     velocity;              /* 目标在车辆坐标系下的X,Y速度，unit：m/s */
    Eigen::Matrix2f     vel_variance;          /* 目标的速度方差  */
    Eigen::Vector2f     acceleration;          /* 目标在车辆坐标系下的X,Y加速度，unit：m/s^2 */ 
    Eigen::Matrix2f     acc_variance;          /* 目标的加速度方差  */
    uint8_t             measurement_status;    /* 测量状态，可能传感器有融合，如RSU可融合后再传递 */
    uint8_t             sensor_type;           /* 传感器类型，融合数据使用主传感器类型 */
    uint8_t             object_type;           /* 目标类别，如车辆、行人等 */
    uint8_t             moving_status;         /* 目标运动状态，如静止、运动、运动中静止等 */
    float               ettc;                  /* 强化距离碰撞时间(ettc，enhanced time to collision)，在加速度不等的情况，unit: s */
    uint32_t            cipv;                  /* 车辆/骑车人被选为cipv flag，若有cipv则置1，若没有cipv则置0 */
    float               curr_lane;             /* 目标所处当前车道，枚举类型详细可见 OBJ_LANE_ASSIGN 描述 */
};


struct alignas(16) SensorFrame
{
    uint32_t    time_stamp;
    SensorType  sensor_type;
    std::vector<SensorObject> sensors;
};


#endif /* D771E8BE_51D2_49D9_B7E0_BF7C56E6C696 */
