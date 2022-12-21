/*
 * @Descripttion: 
 * @version: 
 * @Author: congsir
 * @Date: 2022-12-01 15:11:10
 * @LastEditors: 
 * @LastEditTime: 2022-12-08 09:46:18
 */
#ifndef A77E8904_57C1_47A7_9989_2D1B85029189
#define A77E8904_57C1_47A7_9989_2D1B85029189

#include "data_base.h"
#include <vector>

using std::vector;

class V2xFusionAlgo
{
public:
    /**
     * @brief v2x消息处理总入口
     * @param data v2x消息体
     * @param len v2x消息长度
     */
    static void ProcessRecieveData(uint8_t* data, uint16_t len);
private:
    /**
     * @brief 解析gSentry进程状态消息
     * @param buf gSentry进程状态消息
     * @param len 消息长度
     */
    static void ProcessGSentrySatatus(uint8_t* buf, uint32_t len);
    /**
     * @brief 解析自车补充地图信息
     * @param buf 自车补充地图信息
     * @param len 消息长度
     */
    static void ProcessHostVehiExtraMapInfo(uint8_t* buf, uint32_t len);
    /**
     * @brief 解析v2x交通灯信息
     * @param buf 交通灯信息
     * @param len 消息长度
     */
    static void ProcessSpatInfo(uint8_t* buf, uint32_t len);
    /**
     * @brief 接收v2x远车信息，并将其位置、速度、加速度转换到自车坐标系
     * @param buf 20个远车信息
     * @param len 消息长度
     */
    static void ProcessObjVehiInfo(uint8_t* buf, uint32_t len);
    /**
     * @brief 解析自车地图信息
     * @param buf 自车地图信息
     * @param len 消息长度
     */
    static void ProcessHostVehiMapInfo(uint8_t* buf, uint32_t len);
    /**
     * @brief 解析远车地图信息
     * @param buf 远车地图信息
     * @param len 消息长度
     */
    static void ProcessObjVehiMapInfo(uint8_t* buf, uint32_t len);
    /**
     * @brief 解析gSentry告警信息
     * @param buf gSentry告警信息
     * @param len 消息长度
     */
    static void ProcessGSentryWarningInfo(uint8_t* buf, uint32_t len);
    /**
     * @brief 解析gSentry传递的RSM非机动目标信息，，并将其位置、速度、加速度转换到自车坐标系
     * @param buf 50个RSM非机动目标
     * @param len 消息长度
     */
    static void ProcessRoadPartInfo(uint8_t* buf, uint32_t len);
private:
    /**
     * @brief 将v2x格式的目标车辆，转换为融合格式
     * @param v2x v2x原始数据
     * @param host 自车经纬度
     * @param cdd 融合格式目标
     */
    static void TransV2xVehi2CddVehi(const V2X::AdasObjVehInfo& v2x, const CAN::HostVehiclePos& host, CDD_Fusion_ObjInfo_BUS& cdd);
    /**
     * @brief 将v2x格式的交通灯信息，转换为融合格式
     * @param v2x v2x交通灯信息
     * @param cdd 融合需要的交通灯信息
     */
    static void TransV2xSpat2CddSpat(const V2X::AdasSpatInfo& v2x, CDDFusion::CDDCurntLaneTrafficLightInfo& cdd);
    /**
     * @brief 将v2x格式的告警信息，转换为融合格式
     * @param v2x v2x告警信息
     * @param cdd 融合需要的告警信息
     */
    static void TransV2xWarn2CddWarn(const V2X::WarningInfo& v2x, CDDFusion::CDDgSentryWarningInfo& cdd);
    /**
     * @brief WGS坐标到自车坐标系的转换方法
     * 
     * @param host 自车经纬度
     * @param remote 待转换的远车坐标
     * @return vector<float> 转换后远车的坐标
     */
    static vector<float> TransWgs84ToVcsCoordinate(const CAN::HostVehiclePos& host, const V2X::Position& remote);
    /**
     * @brief 绕Z轴进行坐标旋转方法
     * 
     * @param objXVector 待转换的X
     * @param objYVector 待转化的Y
     * @param hosthead   目标坐标系朝向
     * @param remotehead 当前坐标系朝向
     * @return vector<float> 转换后坐标
     */
    static vector<float> TransObjVcsToHostVcs(const float objXVector, const float objYVector, const float hosthead, const float remotehead);
    static void WirteBack(char* buf, uint16_t len);
};

#endif /* A77E8904_57C1_47A7_9989_2D1B85029189 */
