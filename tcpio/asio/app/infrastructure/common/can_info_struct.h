#ifndef ED3584F6_6AED_436B_BE5C_6F03CDD14CFE
#define ED3584F6_6AED_436B_BE5C_6F03CDD14CFE

namespace CAN
{

struct HostVehiclePos
{
    uint32_t    latitude;           /* 定义纬度数值，北纬为正，南纬为负。offset -180，单位1e-7° */
    uint32_t    longitude;          /* 定义经度数值。东经为正，西经为负。offset -180，单位1e-7° */
    uint32_t    elevation;          /* 车辆海拔，offset -10000，单位0.001m */
    uint16_t    objectHeadingAngle;     /* 目标车辆航向角为运动方向与正北方向的顺时针夹角, offset -360，单位0.010986° */
};

}

#endif /* ED3584F6_6AED_436B_BE5C_6F03CDD14CFE */
