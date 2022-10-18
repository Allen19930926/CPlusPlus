#ifndef A62F6944_1391_4313_B16C_7558850948D8
#define A62F6944_1391_4313_B16C_7558850948D8
#include "sensor_object.h"

class SensorAdapter
{
public:
    static SensorFrame&& Transformation(uint8_t* data);
};

#endif /* A62F6944_1391_4313_B16C_7558850948D8 */
