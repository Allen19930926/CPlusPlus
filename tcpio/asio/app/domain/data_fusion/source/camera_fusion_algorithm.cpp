#include "camera_fusion_algorithm.h"


void CameraFusionAlgo::TransCamera2CddObstacle(const gohigh::Obstacle& camera, const uint32_t timeStamp, CDDFusion::CDDFusionCameraObj& cdd)
{
    cdd.timestamp = timeStamp;
    cdd.id = camera.id;
    cdd.conf = camera.conf;
    cdd.measurementStatus = camera.world_info.measurement_status;
    cdd.lifeTime = camera.life_time;
    cdd.length = camera.world_info.length;
    cdd.width = camera.world_info.width;
    cdd.height = camera.world_info.height;
    cdd.yaw = camera.world_info.yaw;
    cdd.yawConf = camera.world_info.conf_yaw;
    cdd.yawRate = camera.world_info.yaw_rate;
    cdd.dx = camera.world_info.position.x;
    cdd.dy = camera.world_info.position.y;
    cdd.vx = camera.world_info.vel_abs_world.vx;
    cdd.vy = camera.world_info.vel_abs_world.vy;
    cdd.ax = camera.world_info.acc_abs_world.ax;
    cdd.ay = camera.world_info.acc_abs_world.ay;
    // cdd.dxVariance = camera.world_info.sigma_position[0];
    // cdd.dyVariance = camera.world_info.sigma_position[1];
    // cdd.vxVariance = camera.world_info.sigma_vel[0];
    // cdd.vyVariance = camera.world_info.sigma_vel[1];
    // cdd.axVariance
    // cdd.ayVariance
    cdd.currLane = camera.world_info.curr_lane;
    cdd.ttc = camera.world_info.ttc;
    cdd.ettc = camera.world_info.ettc;
    cdd.cipv = camera.world_info.cipv;
}

