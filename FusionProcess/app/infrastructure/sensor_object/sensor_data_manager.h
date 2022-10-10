#ifndef FA3A08FF_9ADF_49BD_9FDD_336756F0C840
#define FA3A08FF_9ADF_49BD_9FDD_336756F0C840

#include "sensor_data.h"
#include <unordered_map>

class SensorDataManager
{
public:
    void AddSensorMeasurements(const SensorFrame& frame);
private:
    using SensorType = uint8_t;
public:
    std::unordered_map<SensorType, Sensor> sensors;
}

#endif /* FA3A08FF_9ADF_49BD_9FDD_336756F0C840 */
