#include "camera_fusion_algorithm.h"
#include <cstring>
#include <queue>


void CameraFusionAlgo::ProcessJ3CameraData(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || len != sizeof(gohigh::Obstacles))
    {
        return ;
    }
    gohigh::Obstacles& obstacles = DataRepo::GetInstance().GetCameraObstacles();
    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    memcpy(&obstacles, buf, len);

    // 使用小顶堆，遍历所有目标障碍物，按和本车距离进行排序，堆顶为最近障碍物
    auto compare = [](gohigh::Obstacle left, gohigh::Obstacle right)
    {
        float lx = left.world_info.position.x;
        float ly = left.world_info.position.y;
        float rx = right.world_info.position.x;
        float ry = right.world_info.position.y;
        return (lx * lx + ly * ly) > (rx * rx + ry * ry);
    };
    std::priority_queue<gohigh::Obstacle, std::vector<gohigh::Obstacle>, decltype(compare)> nearestVehis(compare);
    for (uint16_t i=0; i<obstacles.obstacle_num; i++)
    {
        nearestVehis.push(obstacles.obstacles[i]);
    }

    // 从小顶堆堆顶抛出20个障碍物，填充到fusion结构体
    for (uint16_t i=0; i<CAMERA_OBJ_VEHI_NUM && !nearestVehis.empty(); i++)
    {
        const auto& objectVehicle = nearestVehis.top();
        CameraFusionAlgo::TransCamera2CddObstacle(objectVehicle, obstacles.timestamp, fusion.j3ObjVehi[i]);
        nearestVehis.pop();
    }
}

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

