#ifndef CA074DF7_E7A6_4146_8CCB_A494B3308E05
#define CA074DF7_E7A6_4146_8CCB_A494B3308E05

#include "data_base.h"
#include <vector>
#include <hb_data.h>

class CameraFusionAlgo
{
public:
    /**
     * @brief camera消息处理入口
     * @param data 消息体
     * @param len 消息长度
     * @param msgid 消息ID
     */
    static void ProcessRecieveData(uint8_t* data, uint16_t len, uint16_t msgid) { ProcessJ3CameraData(data, len, msgid); }
    /**
     * @brief 周期执行camera数据融合
     * 
     */
    static void ExecuteCameraDataFusion();
private:
    /**
     * @brief 处理camera消息
     * @param data 消息体
     * @param len 消息长度
     * @param msgid 消息ID
     */
    static void ProcessJ3CameraData(uint8_t* buf, uint16_t len, uint16_t msgid);
    /**
     * @brief 接收存储camera的障碍物消息，按照距离本车欧式距离进行排序
     * 
     * @param buf 障碍物信息（行人或车辆）
     * @param len 消息长度
     */
    static void ProcessCameraObstacles(uint8_t* buf, uint16_t len);
    /**
     * @brief 接收存储camera的车道线信息
     * 
     * @param buf 车道线信息
     * @param len 消息长度
     */
    static void ProcessCameraLines(uint8_t* buf, uint16_t len);
    /**
     * @brief 将camera的障碍物转换为融合格式
     * 
     * @param camera camera的障碍物信息
     * @param cdd 融合需要的目标信息
     */
    static void TransCamera2CddObstacle(const gohigh::Obstacle& camera, CDD_Fusion_ObjInfo_BUS& cdd);
    /**
     * @brief 从本次融合周期内已接收的障碍物中，进行id去重和目标类型筛选，获取最近的20个障碍物
     * 
     * @return 距离本车最近的20个有效障碍物
     */
    static std::vector<gohigh::Obstacle> GetObstaclVecFromHeap();
};

#endif /* CA074DF7_E7A6_4146_8CCB_A494B3308E05 */
