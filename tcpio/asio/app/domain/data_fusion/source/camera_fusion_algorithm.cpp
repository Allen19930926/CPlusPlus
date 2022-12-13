#include "camera_fusion_algorithm.h"
#include <cstring>
#include <queue>
#include <mutex>
#include <algorithm>
#include "bs_debug.h"
#include <atomic>
#include <vector>
#include "event_queue.h"
#include <glog/logging.h>

static void LogOneVehicleInfo(const CDD_Fusion_ObjInfo_Array90& vehicles, const uint32_t num);

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


    const gohigh::Line& GetSpecLine(uint32_t i, const gohigh::Lines& lines)
    {
        if (i == 0)
        {
            LOG(INFO) << "\tpadding left_left lane info";
            return lines.left_left;
        }
        else if (i == 1) 
        {
            LOG(INFO) << "\tpadding left lane info";
            return lines.left;
        }
        else if (i == 2)
        {
            LOG(INFO) << "\tpadding right lane info";
            return lines.right;
        }
            LOG(INFO) << "\tpadding right_right lane info";
        return lines.right_right;
    }

    uint32_t shrink_to_fit(const CDD_Fusion_ObjInfo_Array90& src, CDD_Fusion_ObjInfo_Array90& dst)
    {
        uint32_t valid_cnt = 0;
        for (uint32_t i=0; i<90; i++)
        {
            if (src[i].De_life_time_u32 == 0)
            {
                continue;
            }
            memcpy(&dst[valid_cnt], &src[i], sizeof(CDD_Fusion_ObjInfo_BUS));
            valid_cnt++;
        }
        return valid_cnt;
    }
}

void CameraFusionAlgo::ProcessJ3CameraData(uint8_t* buf, uint16_t len)
{
    if (len == sizeof(gohigh::Obstacles))
    {
        return ProcessCameraObstacles(buf, len);
    }
    else if(len == sizeof(gohigh::Lines))
    {
        return ProcessCameraLines(buf, len);
    }
}

void CameraFusionAlgo::ProcessCameraObstacles(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr)
    {
        return ;
    }
    LOG(INFO) << " recieve J3 object vehicle info";
    gohigh::Obstacles& obstacles = *reinterpret_cast<gohigh::Obstacles*>(buf);

    std::lock_guard<std::mutex> lck(camera_mtx);
    for (uint16_t i=0; i<obstacles.obstacle_num; i++)
    {
        nearestVehis.push(obstacles.obstacles[i]);
    }
}

void CameraFusionAlgo::ProcessCameraLines(uint8_t* buf, uint16_t len)
{
    if (buf == nullptr)
    {
        return ;
    }

    gohigh::Lines lines = *reinterpret_cast<gohigh::Lines*>(buf);
    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();

    for (uint32_t i=0; i<CDD_LANE_INFO_NUM; i++)
    {

        const gohigh::Line& line = GetSpecLine(i, lines);
        fusion.laneInfo[i].De_Timestamp_u32     = lines.timestamp;
        fusion.laneInfo[i].De_LaneValid_u8      = 1;
        fusion.laneInfo[i].De_LaneID_u8         = line.id;
        fusion.laneInfo[i].De_Life_time_u32     = line.life_time;
        fusion.laneInfo[i].De_LaneType_u8       = line.type;
        fusion.laneInfo[i].De_Conf_f32          = line.conf;
        fusion.laneInfo[i].De_LaneWidth_f32     = line.width;
        fusion.laneInfo[i].De_LaneColor_u8      = line.color;
        fusion.laneInfo[i].De_LaneMarking_u8    = line.marking;
        fusion.laneInfo[i].De_start_Xpt_f32     = line.start_pt.x;
        fusion.laneInfo[i].De_start_Ypt_f32     = line.start_pt.y;
        fusion.laneInfo[i].De_LaneDist_f32      = line.t_max;
        fusion.laneInfo[i].De_C0_f32            = line.y_coeff[0];
        fusion.laneInfo[i].De_C1_f32            = line.y_coeff[1];
        fusion.laneInfo[i].De_C2_f32            = line.y_coeff[2];
        fusion.laneInfo[i].De_C3_f32            = line.y_coeff[3];


        LOG(INFO) << "\tDe_Timestamp_u32 :"    << fusion.laneInfo[i].De_Timestamp_u32
                  << "\tDe_LaneValid_u8 :"     << static_cast<uint32_t>(fusion.laneInfo[i].De_LaneValid_u8)
                  << "\tDe_LaneID_u8 :"        << static_cast<uint32_t>(fusion.laneInfo[i].De_LaneID_u8)
                  << "\tDe_Life_time_u32 :"    << fusion.laneInfo[i].De_Life_time_u32
                  << "\n\t\t\tDe_LaneType_u8 :"<< static_cast<uint32_t>(fusion.laneInfo[i].De_LaneType_u8)
                  << "\tDe_Conf_f32 :"         << fusion.laneInfo[i].De_Conf_f32
                  << "\tDe_LaneWidth_f32 :"    << fusion.laneInfo[i].De_LaneWidth_f32
                  << "\tDe_LaneColor_u8 :"     << static_cast<uint32_t>(fusion.laneInfo[i].De_LaneColor_u8)
                  << "\n\t\t\tDe_LaneMarking_u8 :" << static_cast<uint32_t>(fusion.laneInfo[i].De_LaneMarking_u8)
                  << "\tDe_start_Xpt_f32 :"    << fusion.laneInfo[i].De_start_Xpt_f32
                  << "\tDe_start_Ypt_f32 :"    << fusion.laneInfo[i].De_start_Ypt_f32
                  << "\tDe_LaneDist_f32 :"     << fusion.laneInfo[i].De_LaneDist_f32
                  << "\n\t\t\tDe_C0_f32 :"     << fusion.laneInfo[i].De_C0_f32
                  << "\tDe_C1_f32 :"           << fusion.laneInfo[i].De_C1_f32
                  << "\tDe_C2_f32 :"           << fusion.laneInfo[i].De_C2_f32
                  << "\tDe_C3_f32 :"           << fusion.laneInfo[i].De_C3_f32;

    }

    CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_LANE_INFO, reinterpret_cast<const char*>(&fusion.laneInfo), sizeof(fusion.laneInfo)});
}

#ifdef __x86_64__
#include "com_tcp_server.h"
extern ComTcpServer * comTcpServer;
#endif
// 使用异步定时器，处理期间汇聚的行人和车辆信息
void CameraFusionAlgo::ExecuteCameraDataFusion()
{
    std::lock_guard<std::mutex> lck(camera_mtx);

    std::vector<gohigh::Obstacle> tmpVehiVec = GetObstaclVecFromHeap();

    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    memset(&fusion.cddObjects[ADAS_GSENTRY_OBJ_VEHI_NUM + ADAS_GSENTRY_VRU_VEHI_NUM], 0, sizeof(CDD_Fusion_ObjInfo_BUS)*ADAS_CAMERA_OBJ_VEHI_NUM);

    // 从已排序列表取元素进行填充，id相同只取高级别进行填充
    uint32_t i=ADAS_GSENTRY_OBJ_VEHI_NUM + ADAS_GSENTRY_VRU_VEHI_NUM;
    for (const auto& vehicle: tmpVehiVec)
    {
        CameraFusionAlgo::TransCamera2CddObstacle(vehicle, fusion.cddObjects[i]);
        i++;
    }

    CDD_Fusion_ObjInfo_Array90  objVehiInfo;
    memset(&objVehiInfo, 0, sizeof(objVehiInfo));
    uint32_t valid_cnt = shrink_to_fit(fusion.cddObjects, objVehiInfo);

    if (valid_cnt == 0)
    {
        LOG_EVERY_N(ERROR, 20) << "There is no valid vehicle, no need to send !";
        return ;
    }
    LogOneVehicleInfo(objVehiInfo, valid_cnt);

#ifdef __x86_64__
    // 这里先不修改，需要和pytest用例一起修改
    comTcpServer->Write({MsgType::IPC_OBJ_INFO, reinterpret_cast<void*>(&objVehiInfo), sizeof(objVehiInfo)});
    // CDebugFun::PrintBuf(reinterpret_cast<uint8_t*>(&fusion.cddObjects), static_cast<uint16_t>(sizeof(fusion.cddObjects)));
#else
    CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_OBJ_INFO, reinterpret_cast<const char*>(&objVehiInfo), sizeof(objVehiInfo)});
    // LOG(INFO) << "send fusion objects";
#endif
    memset(&fusion.cddObjects[0], 0, sizeof(CDD_Fusion_ObjInfo_Array90));
    LOG(INFO) << "first three vehicle id: " << uint32_t(objVehiInfo[0].De_ID_u8)
              << "    " << uint32_t(objVehiInfo[1].De_ID_u8) << "    " << uint32_t(objVehiInfo[2].De_ID_u8);
}

void CameraFusionAlgo::TransCamera2CddObstacle(const gohigh::Obstacle& camera, CDD_Fusion_ObjInfo_BUS& cdd)
{
    cdd.De_Timestamp_u32            = camera.timestamp;
    cdd.De_ID_u8                    = camera.id + 1;
    cdd.De_conf_f32                 = camera.conf;
    cdd.De_ObjectType_u8            = camera.type;
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
    cdd.De_vx_f32                   = camera.world_info.vel.vx;
    cdd.De_vy_f32                   = camera.world_info.vel.vy;
    cdd.De_ax_f32                   = camera.world_info.acc_ref.ax;
    cdd.De_ay_f32                   = camera.world_info.acc_ref.ay;
    cdd.De_dxVariance_f32           = camera.world_info.sigma_position[0];
    cdd.De_dyVariance_f32           = camera.world_info.sigma_position[4];
    cdd.De_vxVariance_f32           = camera.world_info.sigma_vel[0];
    cdd.De_vyVariance_f32           = camera.world_info.sigma_vel[4];
    cdd.De_axVariance_f32           = 0;
    cdd.De_ayVariance_f32           = 0;
    cdd.De_curr_lane_f32            = camera.world_info.curr_lane;
    cdd.De_ttc_f32                  = camera.world_info.ttc;
    cdd.De_ettc_f32                 = camera.world_info.ettc;
    cdd.De_CIPV_u8                  = camera.world_info.cipv;
    cdd.De_ObjectMovingStatus_u8    = camera.world_info.motion_state;
    cdd.De_source_u8                = 0;
}

void LogOneVehicleInfo(const CDD_Fusion_ObjInfo_Array90& vehicles, const uint32_t num)
{
    for (uint32_t i=0; i<num; i++)
    {
        const CDD_Fusion_ObjInfo_BUS& vehicle = vehicles[i];
        LOG(INFO)   << " Log one vehicle info! "
                    << "\n\tDe_Timestamp_u32 :" << vehicle.De_Timestamp_u32
                    << "\tDe_ObjectType_u8 :" << static_cast<uint32_t>(vehicle.De_ObjectType_u8)
                    << "\tDe_ObjectMovingStatus_u8 :" << static_cast<uint32_t>(vehicle.De_ObjectMovingStatus_u8)
                    << "\tDe_ID_u8 :"         << static_cast<uint32_t>(vehicle.De_ID_u8)
                    << "\n\tDe_source_u8(0:camera  1:v2x) :"       << static_cast<uint32_t>(vehicle.De_source_u8)
                    << "\tDe_Yaw_f32 :"       << vehicle.De_Yaw_f32
                    << "\tDe_CIPV_u8 :"       << static_cast<uint32_t>(vehicle.De_CIPV_u8)
                    << "\n\tDe_dx_f32 :"      << vehicle.De_dx_f32 << "\tDe_dy_f32 :" << vehicle.De_dy_f32
                    << "\n\tDe_vx_f32 :"      << vehicle.De_vx_f32 << "\tDe_vy_f32 :" << vehicle.De_vy_f32
                    << "\n\tDe_ax_f32 :"      << vehicle.De_ax_f32 << "\tDe_ay_f32 :" << vehicle.De_ay_f32;

    }
}

std::vector<gohigh::Obstacle> CameraFusionAlgo::GetObstaclVecFromHeap()
{
    std::vector<gohigh::Obstacle> tmpVehiVec;
    uint32_t i = 0;
    uint32_t lastId = 0xFFFFFFFF;
/*
    从小顶堆堆顶抛出20个障碍物，id相同视为一个障碍物，按照id、类型和时间进行排序，id从小到大排序， id相同时按照类型和时间从大到小排序。
    排序结果示例：
        id           timestamp           type
        56               200              1  
        56               100              2   
        67               200              2 
        67               200              1  
        67               100              2   
        67               100              1 
    最终输出：
        id           timestamp           type
        56               200              1  
        67               200              2  
*/
    while (i<20 && !nearestVehis.empty())
    {
        const auto& objectVehicle = nearestVehis.top();
        tmpVehiVec.push_back(objectVehicle);
        i = (lastId == objectVehicle.id) ? i : i+1;
        nearestVehis.pop();
    }

    std::sort(tmpVehiVec.begin(), tmpVehiVec.end(), [](gohigh::Obstacle& lhs, gohigh::Obstacle& rhs) 
    {
        if (lhs.id != rhs.id)
        {
            return lhs.id < rhs.id;
        }
        else if (lhs.timestamp != rhs.timestamp)
        {
            return lhs.timestamp > rhs.timestamp;
        }
        else
        {
            return lhs.type > rhs.type;
        }
    });

    lastId = 0xFFFFFFFF;
    auto rm_cond = [&lastId](gohigh::Obstacle& vehicle)
    {
        if (vehicle.id == lastId)
        {
            return true;
        }
        lastId = vehicle.id;
        return false;
    };
    tmpVehiVec.erase(std::remove_if(tmpVehiVec.begin(), tmpVehiVec.end(), rm_cond), tmpVehiVec.end());
    return std::move(tmpVehiVec);
}

