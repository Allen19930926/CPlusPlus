#include "prescan_receiver.h"
#include "infrastructure/sensor_object/sensor_object.h"
#include "infrastructure/sensor_object/sensor_data_manager.h"

void PrescanReceiver::PrescanReadCameraMsg(char* data)
{
    SensorFrame frame;
    frame.sensor_type = SensorType::CAMERA;

    PrescanCameraFrame& src = *reinterpret_cast<PrescanCameraFrame*>(data);
    frame.time_stamp = src.CameraHeader.TimeStamp;
    SensorObject obj;
    for (uint32_t i = 0; i < src.CameraHeader.ObjectNum; i++)
    {
        obj.id = src.CameraObjects[i].Object_ID;
        obj.position(0) = src.CameraObjects[i].RangeX;
        obj.position(1) = src.CameraObjects[i].RangeY;
        obj.velocity(0) = src.CameraObjects[i].DopplerVelocityX;
        obj.velocity(1) = src.CameraObjects[i].DopplerVelocityY;
        
        frame.sensors.push_back(obj);
    }

    SensorDataManager::GetInstance().AddSensorMeasurements(frame);
}

void PrescanReceiver::PrescanReadRadarMsg(char* data)
{
    SensorFrame frame;
    frame.sensor_type = SensorType::FRONT_RADAR;
    PrescanRadarFrame& src = *reinterpret_cast<PrescanRadarFrame*>(data);
    frame.time_stamp = src.RadarHeader.TimeStamp;

    SensorObject obj;
    for (uint32_t i = 0; i < src.RadarHeader.ObjectNum; i++)
    {
        obj.id = src.RadarObjects[i].TargetID;
        obj.position(0) = src.RadarObjects[i].TIS_RangeX;
        obj.position(1) = src.RadarObjects[i].TIS_RangeY;
        obj.velocity(0) = src.RadarObjects[i].DopplerVelocityX;
        obj.velocity(1) = src.RadarObjects[i].DopplerVelocityY;

        frame.sensors.push_back(obj);
    }

    SensorDataManager::GetInstance().AddSensorMeasurements(frame);
}