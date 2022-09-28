#ifndef D771E8BE_51D2_49D9_B7E0_BF7C56E6C696
#define D771E8BE_51D2_49D9_B7E0_BF7C56E6C696
#include <cstdint>

struct SensorObject
{
    uint32_t timestamp;
    uint8_t  id;
    float    conf;
    uint8_t  measurement_status;
    uint32_t life_time;
    uint8_t  sensorType;
    uint8_t  objectType;
    uint8_t  moving_status;
    float    length; /* 长，unit：m */
    float    width; /* 宽，unit：m */
    float    height; /* 高，unit：m */
    float    yaw; /* 目标的方位角(朝向)，方向和vcs坐标系一致，左正右负，即逆时针正，顺时针负，unit: rad */
    float    conf_yaw; /* 朝向角置信度[0~100] */
    float    yaw_rate; /* 目标车辆在vcs坐标系下的角速度，unit：rad/s */
    float    dx;
    float    dy;
    float    vx;
    float    vy;
    float    ax;
    float    ay;
    float    dx_variance;
    float    dy_variance;
    float    vx_variance;
    float    vy_variance;
    float    ax_variance;
    float    ay_variance;
    float    curr_lane; /* 目标所处当前车道，枚举类型详细可见 OBJ_LANE_ASSIGN 描述 */
    float    acc; /* 目标在其行驶方向上的对地加速度大小，unit: m/s^2 */
    float    ettc; /* 强化距离碰撞时间(ettc，enhanced time to collision)，在加速度不等的情况，unit: s */
    uint32_t cipv; /* 车辆/骑车人被选为cipv flag，若有cipv则置1，若没有cipv则置0 */
};


#endif /* D771E8BE_51D2_49D9_B7E0_BF7C56E6C696 */
