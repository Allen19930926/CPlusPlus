#include "sensor_adapter.h"
#include "infrastructure/common/ipc_data.h"

bool SensorAdapter::Transformation(uint8_t* data, SensorFrame& v2x_frame, SensorFrame& camera_frame)
{
    CDD_Fusion_ObjInfo_Array40& src = *reinterpret_cast<CDD_Fusion_ObjInfo_Array40*>(data);
    v2x_frame.sensors.clear();
    camera_frame.sensors.clear();

    bool res = true;
    v2x_frame.time_stamp = src[0].De_Timestamp_u32;
    camera_frame.time_stamp = src[0].De_Timestamp_u32;
    v2x_frame.sensor_type = SensorType::V2X;
    camera_frame.sensor_type = SensorType::CAMERA;
    SensorObject obj;
    for (uint32_t i = 0; i < 40; i++)
    {
        if (src[i].De_life_time_u32 == 0)
        {
            break;
        }
        res = false;
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

        if (obj.sensor_type == 0)
        {
            camera_frame.sensors.push_back(obj);
            continue;
        }
        v2x_frame.sensors.push_back(obj);
    }
    return res;
}

