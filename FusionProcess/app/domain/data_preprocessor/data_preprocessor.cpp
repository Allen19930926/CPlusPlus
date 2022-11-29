#include "domain/data_preprocessor/data_preprocessor.h"
#include "infrastructure/sensor_object/sensor_object.h"
#include "infrastructure/sensor_object/sensor_data_manager.h"
#include "infrastructure/common/hb_data.h"
#include "infrastructure/common/ipc_data.h"

DataPreprocessor::DataPreprocessor() : func_table(SensorType::MAX, nullptr)
{
    func_table[SensorType::CAMERA] = &DataPreprocessor::ProcessCameraMsg;
    func_table[SensorType::V2X] = &DataPreprocessor::ProcessV2xMsg;
}

void DataPreprocessor::ProcessReadMsg(FusionChatMessage msg)
{
    uint32_t type = msg.Header().sensor_type;
    if (type >= SensorType::MAX)
    {
        return ;
    }

    if (func_table[type] == nullptr)
    {
        return ;
    }

    (this->*func_table[type])(msg.Body(), msg.BodyLength());
}

void DataPreprocessor::ProcessV2xMsg(char* data, uint32_t len)
{
    if (len != 20 * sizeof(CDD_Fusion_ObjInfo_BUS))
    {
        return ;
    }

    SensorFrame frame;
    frame.sensor_type = SensorType::V2X;
    CDD_Fusion_ObjInfo_Array20& src = *reinterpret_cast<CDD_Fusion_ObjInfo_Array20*>(data);

    SensorObject obj;
    for (uint32_t i = 0; i < 20; i++)
    {
        if (src[i].De_life_time_u32 == 0)
        {
            break;
        }
        obj.time_stamp          = src[i].De_Timestamp_u32;
        obj.id                  = src[i].De_ID_u8;
        obj.conf                = src[i].De_conf_f32;
        obj.sensor_type         = src[i].De_source_u8;
        obj.life_time           = src[i].De_life_time_u32;
        obj.yaw                 = src[i].De_Yaw_f32;
        obj.conf_yaw            = src[i].De_conf_yaw_f32;
        obj.yaw_rate            = src[i].De_yaw_rate_f32;
        obj.size(0)             = src[i].De_length_f32;
        obj.size(1)             = src[i].De_width_f32;
        obj.size(2)             = src[i].De_height_f32;
        obj.position(0)         = src[i].De_dx_f32;
        obj.position(1)         = src[i].De_dy_f32;
        obj.pos_variance(0,0)   = src[i].De_dxVariance_f32;
        obj.pos_variance(1,1)   = src[i].De_dyVariance_f32;
        obj.velocity(0)         = src[i].De_vx_f32;
        obj.velocity(1)         = src[i].De_vy_f32;
        obj.vel_variance(0,0)   = src[i].De_vxVariance_f32;
        obj.vel_variance(1,1)   = src[i].De_vyVariance_f32;
        obj.acceleration(0)     = src[i].De_ax_f32;
        obj.acceleration(1)     = src[i].De_ay_f32;
        obj.acc_variance(0,0)   = src[i].De_axVariance_f32;
        obj.acc_variance(1,1)   = src[i].De_ayVariance_f32;
        obj.measurement_status  = src[i].De_measurement_status_u8;
        obj.object_type         = src[i].De_ObjectType_u8;
        obj.moving_status       = src[i].De_ObjectMovingStatus_u8;
        obj.ettc                = src[i].De_ettc_f32;
        obj.cipv                = src[i].De_CIPV_u8;

        frame.time_stamp        = obj.time_stamp;
        frame.sensors.push_back(obj);
    }

    SensorDataManager::GetInstance().AddSensorMeasurements(frame);
}

void DataPreprocessor::ProcessCameraMsg(char* data, uint32_t len)
{
    if (len != sizeof(gohigh::Obstacles))
    {
        return ;
    }

    const gohigh::Obstacles& obstacles = *reinterpret_cast<gohigh::Obstacles*>(data);
    SensorFrame frame;
    frame.sensor_type = SensorType::CAMERA;

    SensorObject obj;
    for (uint32_t i=0; i<obstacles.obstacle_num; i++)
    {
        obj.time_stamp          = obstacles.obstacles[i].timestamp;
        obj.id                  = obstacles.obstacles[i].id;
        obj.conf                = obstacles.obstacles[i].conf;
        obj.sensor_type         = 0;
        obj.life_time           = obstacles.obstacles[i].life_time;
        obj.yaw                 = obstacles.obstacles[i].world_info.yaw;
        obj.conf_yaw            = obstacles.obstacles[i].world_info.conf_yaw;
        obj.yaw_rate            = obstacles.obstacles[i].world_info.yaw_rate;
        obj.size(0)             = obstacles.obstacles[i].world_info.length;
        obj.size(1)             = obstacles.obstacles[i].world_info.width;
        obj.size(2)             = obstacles.obstacles[i].world_info.height;
        obj.position(0)         = obstacles.obstacles[i].world_info.position.x;
        obj.position(1)         = obstacles.obstacles[i].world_info.position.y;
        obj.pos_variance(0,0)   = obstacles.obstacles[i].world_info.sigma_position[0];
        obj.pos_variance(1,1)   = obstacles.obstacles[i].world_info.sigma_position[4];
        obj.velocity(0)         = obstacles.obstacles[i].world_info.vel.vx;
        obj.velocity(1)         = obstacles.obstacles[i].world_info.vel.vy;
        obj.vel_variance(0,0)   = obstacles.obstacles[i].world_info.sigma_vel[0];
        obj.vel_variance(1,1)   = obstacles.obstacles[i].world_info.sigma_vel[4];
        obj.acceleration(0)     = obstacles.obstacles[i].world_info.acc_ref.ax;
        obj.acceleration(1)     = obstacles.obstacles[i].world_info.acc_ref.ay;
        obj.acc_variance(0,0)   = 0;
        obj.acc_variance(1,1)   = 0;
        obj.measurement_status  = obstacles.obstacles[i].world_info.measurement_status;
        obj.object_type         = obstacles.obstacles[i].type;
        obj.moving_status       = obstacles.obstacles[i].world_info.motion_state;
        obj.ettc                = obstacles.obstacles[i].world_info.ettc;
        obj.cipv                = obstacles.obstacles[i].world_info.cipv;
    }
    
    SensorDataManager::GetInstance().AddSensorMeasurements(frame);
}

