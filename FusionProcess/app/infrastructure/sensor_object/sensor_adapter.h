#ifndef A62F6944_1391_4313_B16C_7558850948D8
#define A62F6944_1391_4313_B16C_7558850948D8
#include "sensor_object.h"

class SensorAdapter
{
public:
    /**
     * @brief 传感器数据类型转换，BS->fusion
     * @param data bs软件传递的传感器数据，包含camera和v2x
     * @param v2x_frame 转换后的v2x传感器数据帧
     * @param camera_frame 转换后的camera传感器数据帧
     * @return 转换结果
     */
    static bool Transformation(uint8_t* data, SensorFrame& v2x_frame, SensorFrame& camera_frame);
};

#endif /* A62F6944_1391_4313_B16C_7558850948D8 */
