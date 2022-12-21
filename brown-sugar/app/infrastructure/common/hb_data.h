#ifndef HB_DATA_H_
#define HB_DATA_H_

//#pragma pack(push, 4)

#define MAX_OBSTACLE_NUM 50

typedef unsigned int uint32;
// typedef long long int64;

namespace gohigh
{
    typedef struct
    {
        float x;
        float y;
    } Point;

    typedef struct
    {
        bool valid;
        uint32 id;        /* 感知对象的 ID  */
        uint32 life_time; /* 跟踪时长，指车道线的生命周期，存活时间 */
        uint32 type;      /* LineType:Unknown, LaneLine, Curb, Center, Guardrail, ConcreteBarrier, Fence, Wall, Canopy, Virtual, Cone */
        float conf;       /* 线置信度(0-1) */
        /* CurveLine */
        float width;        /* curve width */
        Point start_pt;     /* start point for ADAS curve */
        float y_coeff[4];   /* y = y_coeff[0] + y_coeff[1] * x + y_coeff[2] * x^2 + y_coeff[3] * x^3 */
        float t_max;        /* vcs下曲线x方向长度 */
        uint32 color;       /* LineColor */
        uint32 marking;     /* LineMarking */
        float parsing_conf; /* 表征车道线的观测质量 */
        float rmse;         /* 观测（采样点）和线的均方误差 */
    } Line;

    typedef struct
	{
		uint32 timestamp;
        Line left;
        Line left_left;
        Line right;
        Line right_right;
        float dtlc; /* 前轮外侧到车道线内沿横向距离/本车横向速度，单位 m */
        float ttlc; /* 穿行时距，前轮外侧到车道线内沿横向距离/本车横向速度，单位 s */
    } Lines;


	typedef struct
	{
		float vx;
		float vy;
	} Velocity;

	typedef struct
	{
		float ax;
		float ay;
	} Accleration;

	typedef struct
	{
		float objCornerPoint_x;
		float objCornerPoint_y;
		float objDistInLane;
		uint32 objCutInFlag;
		uint32 objCutInLane;
		uint32 ll_type;
        uint32 rr_type;
		float distance_to_ll;
		float distance_to_rr;
	} ObjCornerPoint;

    typedef struct
    {
		float yaw; /* 目标的方位角(朝向)，方向和vcs坐标系一致，左正右负，即逆时针正，顺时针负，unit: rad */
        Velocity vel; /* 目标在vcs下的相对速度，unit：m/s */
        float length; /* 长，unit：m */
        float width; /* 宽，unit：m */
        float height; /* 高，unit：m */
		Point position; /* 目标在vcs下的相对位置信息(滤波后)，unit：m */
        float ttc; /* 碰撞时间(ttc，time to collision)，两车相对距离/相对车速，unit: s */
		uint32 curr_lane; /* 目标所处当前车道，枚举类型详细可见 OBJ_LANE_ASSIGN 描述 */
        float ettc; /* 强化距离碰撞时间(ettc，enhanced time to collision)，在加速度不等的情况，unit: s */
        float acc; /* 目标在其行驶方向上的对地加速度大小，unit: m/s^2 */
		uint32 motion_state; /* 目标的位置状态，枚举类型详细可见 ObstacleMotionState 描述 */
        Velocity vel_abs_world; /* 目标在vcs方向上的对地速度，unit：m/s */
		Accleration acc_abs_world; /* 目标在vcs方向上的对地加速度，unit：m/s^2 */
		uint32 motion_category; /* 目标车辆的运行类别，枚举类型详细可见 MOTION_CATEGORY 描述 */
		uint32 position_type; /* 目标车辆测距点类型，即目标被检测为车辆时在vcs下的位置类型，枚举类型详细可见 WorldSpaceInfoPositionType 描述 */
        float yaw_rate; /* 目标车辆在vcs坐标系下的角速度，unit：rad/s */
        float sigma_yaw; /* 方位角方差(统计量) */
        float sigma_vel[9]; /* 速度方差3*3(统计量) */
        float sigma_width; /* 宽度方差(统计量) */
        float sigma_height; /* 高度方差(统计量) */
        float sigma_position[9]; /* 位置方差3*3(统计量) */
        float sigma_length; /* 长度方差(统计量) */
		float conf_yaw; /* 朝向角置信度[0~100] */
		uint32 cipv; /* 车辆/骑车人被选为cipv flag，若有cipv则置1，若没有cipv则置0 */
		uint32 measurement_status; /* 可预测状态，目标是否为预测得到的目标，枚举类型详细描述如下: enum ObstacleMeasurementStatus { ​ Obs_Measure_Invalid = 0; //无效的 ​ Obs_Measure_Measured = 1; //检测到的 ​ Obs_Measure_Perdicted = 2; //预测的 } */
		float mid_angle; /* 目标车辆车尾底边中点相对于自车中轴线的角度，方向和vcs坐标系一致，左正右负，即逆时针正，顺时针负，unit：rad */
        ObjCornerPoint obj_corner_point; /* 目标车辆在vcs下cut in时的角点属性信息，详细可见 ObjCornerPoint 描述 */
		Accleration acc_ref; /* 目标车辆的相对加速度，unit：m/s^2 */
		uint32 mcp; /* 行人被选取为紧急行人flag，若有mcp则置1，若没有mcp则置0 */
    }WorldSpaceInfo;

    typedef struct
    {
        uint32 id;
        uint32 timestamp;
        uint32 type;
        uint32 conf;
        uint32 life_time;
        WorldSpaceInfo world_info;
        uint32 serial_number; /* 关键目标选择方法：以车道划分，使用serial_number标识，枚举类型详细可见 ObjectSelectLevel 描述 */
        uint32 select_level; /* 关键目标选择方法：以目标vcs下位置进行划分，使用select_level标识，枚举类型详细可见 ObjectSelectLevel 描述 */
    }Obstacle;

    typedef struct
    {
        uint32 obstacle_num;
        Obstacle obstacles[MAX_OBSTACLE_NUM];
        uint32 cipv_id;
        uint32 mcp_id;
    }Obstacles;
}
//#pragma pack(pop)

#endif // !HB_DATA_H_