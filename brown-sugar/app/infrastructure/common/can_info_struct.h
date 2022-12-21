#ifndef ED3584F6_6AED_436B_BE5C_6F03CDD14CFE
#define ED3584F6_6AED_436B_BE5C_6F03CDD14CFE

namespace CAN
{

struct HostVehiclePos
{
    HostVehiclePos(): latitude(0), longitude(0), elevation(0), objectHeadingAngle(0), isHostPosValid(false) {}
    double    latitude;           /* 定义纬度数值，北纬为正，南纬为负 */
    double    longitude;          /* 定义经度数值。东经为正，西经为负 */
    float     elevation;          /* 车辆海拔 */
    float     objectHeadingAngle;     /* 目标车辆航向角为运动方向与正北方向的顺时针夹角*/
    float     speed;
    float     acc_x;
    float     acc_y;
    bool      isHostPosValid;
} __attribute__((packed, aligned(1)));

}

#endif /* ED3584F6_6AED_436B_BE5C_6F03CDD14CFE */
