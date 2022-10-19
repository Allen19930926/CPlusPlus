#include "sensor_adapter.h"
#include "ipc_data.h"

bool SensorAdapter::Transformation(uint8_t* data, SensorFrame& frame)
{
    CDD_Fusion_ObjInfo_Array40&  src = *reinterpret_cast<CDD_Fusion_ObjInfo_Array40*>(data);
    frame.sensors.clear();

    bool res = false;
    frame.time_stamp  = src[0].De_Timestamp_u32;
    frame.sensor_type = static_cast<SensorType>(src[0].De_source_u8);
    for (uint32_t i=0; i<20; i++)
    {
        if (src[i].De_life_time_u32 == 0)
        {
            break;
        }
        res = true;
        SensorObject obj;
        obj.time_stamp            	= src[i].De_Timestamp_u32;
        obj.id            			= src[i].De_ID_u8;
        obj.conf            		= src[i].De_conf_f32;
        obj.sensor_type             = src[i].De_source_u8;
        obj.life_time            	= src[i].De_life_time_u32;
        obj.yaw           		 	= src[i].De_Yaw_f32;
        obj.conf_yaw            	= src[i].De_conf_yaw_f32;
        obj.yaw_rate            	= src[i].De_yaw_rate_f32;
        obj.size(0)            		= src[i].De_length_f32;
        obj.size(1)            		= src[i].De_width_f32;
        obj.size(2)            		= src[i].De_height_f32;
        obj.position(0)            	= src[i].De_dx_f32;
        obj.position(1)            	= src[i].De_dy_f32;
        obj.pos_variance(0)         = src[i].De_dxVariance_f32;
        obj.pos_variance(1)         = src[i].De_dyVariance_f32;
        obj.velocity(0)            	= src[i].De_vx_f32;
        obj.velocity(1)            	= src[i].De_vy_f32;
        obj.vel_variance(0)         = src[i].De_vxVariance_f32;
        obj.vel_variance(1)         = src[i].De_vyVariance_f32;
        obj.acceleration(0)         = src[i].De_ax_f32;
        obj.acceleration(1)         = src[i].De_ay_f32;
        obj.acc_variance(0)         = src[i].De_axVariance_f32;
        obj.acc_variance(1)         = src[i].De_ayVariance_f32;
        obj.measurement_status     	= src[i].De_measurement_status_u8;
        obj.object_type            	= src[i].De_ObjectType_u8;
        obj.moving_status           = src[i].De_ObjectMovingStatus_u8;
        obj.ettc            		= src[i].De_ettc_f32;
        obj.cipv            		= src[i].De_CIPV_u8;
        frame.sensors.push_back(obj);
    }
    return res;
}

