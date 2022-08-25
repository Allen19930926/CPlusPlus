#include "camera_fusion_algorithm.h"
#include <cstring>
#include <queue>
#include <mutex>
#include <algorithm>
#include "bs_debug.h"
#include <atomic>

namespace
{
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

    std::mutex camera_mtx;
    std::atomic_bool isRecieveMsg(false);
}

void CameraFusionAlgo::ProcessJ3CameraData(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr || len != sizeof(gohigh::Obstacles))
    {
        return ;
    }
    gohigh::Obstacles& obstacles = DataRepo::GetInstance().GetCameraObstacles();
    memcpy(&obstacles, buf, len);

    std::lock_guard<std::mutex> lck(camera_mtx);
    for (uint16_t i=0; i<obstacles.obstacle_num; i++)
    {
        nearestVehis.push(obstacles.obstacles[i]);
    }
    isRecieveMsg.store(true);
}

#ifdef __x86_64__
#include "com_tcp_server.h"
extern ComTcpServer * comTcpServer;
#endif
// 使用异步定时器，处理期间汇聚的行人和车辆信息
void CameraFusionAlgo::ExecuteCameraDataFusion()
{
    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();

    std::lock_guard<std::mutex> lck(camera_mtx);
    // 从小顶堆堆顶抛出20个障碍物，填充到fusion结构体
    for (uint16_t i=ADAS_GSENTRY_OBJ_VEHI_NUM; i<(ADAS_GSENTRY_OBJ_VEHI_NUM + ADAS_CAMERA_OBJ_VEHI_NUM) && !nearestVehis.empty(); i++)
    {
        const auto& objectVehicle = nearestVehis.top();
        CameraFusionAlgo::TransCamera2CddObstacle(objectVehicle, fusion.cddObjects[i]);
        nearestVehis.pop();
    }

#ifdef __x86_64__
    if (isRecieveMsg.load())
    {
        comTcpServer->Write({MsgType::CDD_CAMERA, reinterpret_cast<void*>(&fusion.cddObjects), sizeof(fusion.cddObjects)});
        // CDebugFun::PrintBuf(reinterpret_cast<uint8_t*>(&fusion.cddObjects), static_cast<uint16_t>(sizeof(fusion.cddObjects)));
    }
#endif
    isRecieveMsg.store(false);
    if (isRecieveMsg.load())
    {
        printf("load failed");
    }

}

void CameraFusionAlgo::TransCamera2CddObstacle(const gohigh::Obstacle& camera, CDD_Fusion_ObjInfo_BUS& cdd)
{
    cdd.De_Timestamp_u32			= camera.timestamp;
    cdd.De_ID_u8                    = camera.id;
    cdd.De_conf_f32                 = camera.conf;
    cdd.De_measurement_status_u8    = camera.world_info.measurement_status;
    cdd.De_life_time_u32            = camera.life_time;
    cdd.De_length_f32               = camera.world_info.length;
    cdd.De_width_f32                = camera.world_info.width;
    cdd.De_height_f32               = camera.world_info.height;
    cdd.De_Yaw_f32                  = camera.world_info.yaw;
    cdd.De_conf_yaw_f32             = camera.world_info.conf_yaw;
    cdd.De_yaw_rate_f32             = camera.world_info.yaw_rate;
    cdd.De_dx_f32                   = camera.world_info.position.x;
    cdd.De_dy_f32                   = camera.world_info.position.y;
    cdd.De_vx_f32                   = camera.world_info.vel_abs_world.vx;
    cdd.De_vy_f32                   = camera.world_info.vel_abs_world.vy;
    cdd.De_ax_f32                   = camera.world_info.acc_abs_world.ax;
    cdd.De_ay_f32                   = camera.world_info.acc_abs_world.ay;
    // cdd.De_dxVariance_f32           = camera.world_info.sigma_position[0];
    // cdd.De_dyVariance_f32           = camera.world_info.sigma_position[1];
    // cdd.De_vxVariance_f32           = camera.world_info.sigma_vel[0];
    // cdd.De_vyVariance_f32           = camera.world_info.sigma_vel[1];
    // cdd.De_axVariance_f32           
    // cdd.De_ayVariance_f32           
    cdd.De_curr_lane_f32            = camera.world_info.curr_lane;
    cdd.De_ttc_f32                  = camera.world_info.ttc;
    cdd.De_ettc_f32                 = camera.world_info.ettc;
    cdd.De_CIPV_u8                  = camera.world_info.cipv;
    cdd.De_source_u32               = 0;
}

