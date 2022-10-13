#include "v2x_fusion_algorithm.h"
#include "v2x_adas_event_macro.h"
#include <cmath>
#include <cstring>
#include <glog/logging.h>
#include "Eigen/Dense"
#include "proxy_repository.h"
#include "gSentry_proxy.h"
#include "bs_debug.h"
#include "event_queue.h"

namespace
{
    const double PI = 3.141592653589793;        //double类型精度为小数后16位
    const double V2X_CAN_DEGREE_DIAMETER = 1E-7;
    const double CAN_DEGREE_OFFSET = 180;
    const double V2X_ELEVATION_DIAMETER = 0.1;
    const double V2X_VELOCITY_DIAMETER = 0.02;
    const double V2X_ACCELERATION_DIAMETER = 0.01;
    const double CAN_ELEVATION_DIAMETER = 0.001;
    const double CAN_ELEVATION_OFFSET = 10000;
    const double CAN_HEAD_ANGLE_DIAMETER = 0.010986;
    const double CAN_HEAD_ANGLE_OFFSET = 360;
    const int PI_DEGREE = 180;
    const double MAJOR_AXIS = 6378137.0;
    const double MINOR_AXIS = 6356752.3142;
    const double ee = (MAJOR_AXIS + MINOR_AXIS) * (MAJOR_AXIS - MINOR_AXIS) / (MAJOR_AXIS * MAJOR_AXIS);
}

void V2xFusionAlgo::ProcessRecieveData(uint8_t* data, uint16_t len)
{
    LOG(INFO) << "enter V2xFusionAlgo::ProcessRecieveData";
    if (len < sizeof(V2X::V2xAdasMsgHeader) || data == nullptr)
    {
        return ;
    }

    const V2X::V2xAdasMsgHeader& head = *reinterpret_cast<V2X::V2xAdasMsgHeader*>(data);
    uint8_t* payload = data + sizeof(V2X::V2xAdasMsgHeader);
    uint32_t length  = head.msgLen;

    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    LOG(INFO) << "V2xFusionAlgo process recv data, msgId:" << head.msgId << " fault status:" << v2xData.status.faultStatus;
    if ((head.msgId != EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT) && v2xData.status.faultStatus)
    {
        return ;
    }

    switch(head.msgId)
    {
        case EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT:     ProcessGSentrySatatus(payload, length); break;
        case EV_GSENTRY_ADAS_CALC_MAPINFO_REPORT:       ProcessHostVehiExtraMapInfo(payload, length); break;
        case EV_GSENTRY_ADAS_SPATINFO_REPORT:           ProcessSpatInfo(payload, length); break;
        case EV_GSENTRY_ADAS_OBJECT_VEHICLE_REPORT:     ProcessObjVehiInfo(payload, length); break;
        case EV_GSENTRY_ADAS_EGOVEHI_MAPINFO_REPORT:    ProcessHostVehiMapInfo(payload, length); break;
        case EV_GSENTRY_ADAS_OBJVEHI_MAPINFO_REPORT:    ProcessObjVehiMapInfo(payload, length); break;
        case EV_GSENTRY_ADAS_WARNING_REPORT:            ProcessGSentryWarningInfo(payload, length); break;
        default: break;
    }
}

/* bussiness data extract */

void V2xFusionAlgo::ProcessGSentrySatatus(uint8_t* buf, uint32_t len)
{
    if (len != sizeof(V2X::GSentryStatus))
    {
        return ;
    }
    const V2X::GSentryStatus gSentryStatus = *reinterpret_cast<V2X::GSentryStatus*>(buf);
    LOG(INFO) << "--------------------ProcessGSentrySatatus------------------- " ;
    LOG(INFO) << "1 gSentryStatus : " << int(gSentryStatus.gSentryStatus);
    LOG(INFO) << "2 faultStatus : " << int(gSentryStatus.faultStatus);
    LOG(INFO) << "------------------------------------------------------------ " ;
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    v2xData.status.faultStatus = gSentryStatus.faultStatus;
    v2xData.status.gSentryStatus = gSentryStatus.gSentryStatus;
    WirteBack(reinterpret_cast<char*>(&v2xData.status), static_cast<uint16_t>(sizeof(V2X::GSentryStatus)));
}

void V2xFusionAlgo::ProcessHostVehiExtraMapInfo(uint8_t* buf, uint32_t len)
{
    if (len != sizeof(V2X::MapAddResult))
    {
        LOG(INFO) << "--------------------ProcessHostVehiExtraMapInfo------------------- " ;
        LOG(INFO) << "len != sizeof(V2X::MapAddResult) return " ;
        LOG(INFO) << "len  " << len;
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.mapAddRes, buf, len);
    LOG(INFO) << "--------------------ProcessHostVehiExtraMapInfo------------------- " ;
    LOG(INFO) << "1 offsetTolink : " << v2xData.mapAddRes.offsetTolink;
    LOG(INFO) << "2 offsetTolane : " << v2xData.mapAddRes.offsetTolane;
    LOG(INFO) << "3 distToNode : " << v2xData.mapAddRes.distToNode;
    LOG(INFO) << "4 isAtAcross : " << (int)v2xData.mapAddRes.isAtAcross;
    LOG(INFO) << "------------------------------------------------------------------ " ;
    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    fusion.disToEndLane.disToEndLane = v2xData.mapAddRes.distToNode;
	CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_DIS2ENDLANE, (const char *)(&fusion.disToEndLane), sizeof(fusion.disToEndLane)});
}

void V2xFusionAlgo::ProcessSpatInfo(uint8_t* buf, uint32_t len)
{
    if (len != sizeof(V2X::AdasSpatInfo) * ADAS_SPAT_INFO_NUM)
    {
        LOG(INFO) << "--------------------ProcessSpatInfo------------------- " ;
        LOG(INFO) << "len != sizeof(V2X::AdasSpatInfo) * ADAS_SPAT_INFO_NUM return " ;
        LOG(INFO) << "len  " << len;
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.spatInfo, buf, len);
    for(int i = 0; i<5;i++)
    {
    LOG(INFO) << "--------------------ProcessSpatInfo------------------- " ;
    LOG(INFO) << "1 spatInfo["<< i<< "].timeStamp : " << v2xData.spatInfo[i].timeStamp;
    LOG(INFO) << "2 spatInfo["<< i<< "].spatInfoValid : " << (int)v2xData.spatInfo[i].spatInfoValid;
    LOG(INFO) << "3 spatInfo["<< i<< "].belongsNodeId : " << v2xData.spatInfo[i].belongsNodeId;
    LOG(INFO) << "4 spatInfo["<< i<< "].curCoutingTime : " << v2xData.spatInfo[i].curCoutingTime;
    LOG(INFO) << "5 spatInfo["<< i<< "].nextLight : " << (int)v2xData.spatInfo[i].nextLight;
    LOG(INFO) << "5 spatInfo["<< i<< "].nextDurationTime : " << v2xData.spatInfo[i].nextDurationTime;
    LOG(INFO) << "------------------------------------------------------------------ " ;
    }
    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    // 融合数据只用了当前车道下一个红绿灯信息，因此只取v2x数据的第一个红绿灯信息
    TransV2xSpat2CddSpat(v2xData.spatInfo[0], fusion.spatInfo);
    WirteBack(reinterpret_cast<char*>(&fusion.spatInfo), static_cast<uint16_t>(sizeof(CDDFusion::CDDCurntLaneTrafficLightInfo)));
	CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_TRAFFIC_LIGHT_INFO, (const char *)(&fusion.spatInfo), sizeof(fusion.spatInfo)});
}

void V2xFusionAlgo::ProcessObjVehiInfo(uint8_t* buf, uint32_t len)
{
    if (len != sizeof(V2X::AdasObjVehInfo) * ADAS_OBJ_VEH_INFO_NUM)
    {
        LOG(INFO) << "--------------------ProcessObjVehiInfo------------------- " ;
        LOG(INFO) << "len != sizeof(V2X::AdasObjVehInfo) * ADAS_OBJ_VEH_INFO_NUM return " ;
        LOG(INFO) << "len  " << len;
        LOG(INFO) << "sizeof(V2X::AdasObjVehInfo) * ADAS_OBJ_VEH_INFO_NUM = " << sizeof(V2X::AdasObjVehInfo) * ADAS_OBJ_VEH_INFO_NUM;
        return ;
    }

    CAN::HostVehiclePos& host = DataRepo::GetInstance().GetHostVehicle();
    if (!host.isHostPosValid)
    {
        //return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.objVehicle[0], buf, len);
    for(int i = 0; i < ADAS_OBJ_VEH_INFO_NUM; i++)
    {
        LOG(INFO) << "reveh[" << i << "]--------------------ProcessObjVehiInfo------------------- " ;
        LOG(INFO) << "1 timeStamp : " << v2xData.objVehicle[i].timeStamp;
        LOG(INFO) << "2 localId : " << (int)v2xData.objVehicle[i].localId;
        LOG(INFO) << "3 objectSource : " << (int)v2xData.objVehicle[i].objectSource;
        LOG(INFO) << "4 vehicleClass : " << (int)v2xData.objVehicle[i].vehicleClass;
        LOG(INFO) << "5 length : " << v2xData.objVehicle[i].size.length;
        LOG(INFO) << "6 width : " << v2xData.objVehicle[i].size.width;
        LOG(INFO) << "7 height : " << (int)v2xData.objVehicle[i].size.height;
        LOG(INFO) << "8 objectHeadingAngle : " << v2xData.objVehicle[i].objectHeadingAngle;
        LOG(INFO) << "9 objectYawAngle : " << v2xData.objVehicle[i].objectYawAngle;
        LOG(INFO) << "10 gear : " << (int)v2xData.objVehicle[i].gear;
        LOG(INFO) << "11 steeringWheelAngle : " << (int)v2xData.objVehicle[i].steeringWheelAngle;
        LOG(INFO) << "12 remoteLight : " << (int)v2xData.objVehicle[i].remoteLight;
        LOG(INFO) << "13 brakePedalStatus : " << (int)v2xData.objVehicle[i].vehicleBrakes.brakePedalStatus;
        LOG(INFO) << "14 ABS : " << (int)v2xData.objVehicle[i].vehicleBrakes.absBrakes;
        LOG(INFO) << "15 brakeAppliedStatus : " << (int)v2xData.objVehicle[i].vehicleBrakes.brakeAppliedStatus;
        LOG(INFO) << "16 brakeBoostApplied : " << (int)v2xData.objVehicle[i].vehicleBrakes.brakeBoostApplied;
        LOG(INFO) << "16 auxiliaryBrakeStatus : " << (int)v2xData.objVehicle[i].vehicleBrakes.auxiliaryBrakeStatus;
        LOG(INFO) << "------------------------------------------------------------------ " ;
    }

    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();

    uint16_t i = 0;
    v2xData.objVehiNum = 0;
    while(v2xData.objVehicle[i].localId != 0 && i < ADAS_OBJ_VEH_INFO_NUM && i < ADAS_GSENTRY_OBJ_VEHI_NUM)
    {
        TransV2xVehi2CddVehi(v2xData.objVehicle[i], host, fusion.cddObjects[i]);
        // printf("recieve gSentry ObjVehiInfo msg!\n");
        v2xData.objVehiNum++;
        i++;
    }
    WirteBack(reinterpret_cast<char*>(&fusion.cddObjects), static_cast<uint16_t>(sizeof(fusion.cddObjects)));
}

void V2xFusionAlgo::ProcessHostVehiMapInfo(uint8_t* buf, uint32_t len)
{
    if (len != sizeof(V2X::EgoVehMapInfo))
    {
        LOG(INFO) << "--------------------ProcessHostVehiMapInfo------------------- " ;
        LOG(INFO) << "len != sizeof(V2X::EgoVehMapInfo) " ;
        LOG(INFO) << "len  " << len;
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.egoMap, buf, len);


    LOG(INFO) << "--------------------ProcessHostVehiMapInfo------------------- " ;
    LOG(INFO) << "1 curLink.upstreamNode.RegionId : " << v2xData.egoMap.curLink.upstreamNode.RegionId;
    LOG(INFO) << "2 curLink.upstreamNode.id : " << v2xData.egoMap.curLink.upstreamNode.id;
    LOG(INFO) << "3 curLink.downNode : " << v2xData.egoMap.curLink.downNode;
    LOG(INFO) << "4 curLink.linkWidth : " << v2xData.egoMap.curLink.linkWidth;
    LOG(INFO) << "5 curLink.speedLimit.minSpeed : " << v2xData.egoMap.curLink.speedLimit.minSpeed;
    LOG(INFO) << "6 curLink.speedLimit.maxSpeed : " << v2xData.egoMap.curLink.speedLimit.maxSpeed;
    LOG(INFO) << "7 curLane.laneID : " << (int)v2xData.egoMap.curLane.laneID;
    LOG(INFO) << "8 curLane.laneWidth : " << v2xData.egoMap.curLane.laneWidth;
    LOG(INFO) << "9 curLane.maneuvers : " << v2xData.egoMap.curLane.maneuvers;
    LOG(INFO) << "10 curLane.lanePointNextNo : " << (int)v2xData.egoMap.curLane.lanePointNextNo;
    LOG(INFO) << "11 downLinksList[0].id : " << (int)v2xData.egoMap.downLinksList[0].id;
    LOG(INFO) << "12 downLinksList[0].linkWidth : " << v2xData.egoMap.downLinksList[0].linkWidth;
    LOG(INFO) << "13 downLinksList[0].downLanesList[0].id : " << (int)v2xData.egoMap.downLinksList[0].downLanesList[0].id;
    LOG(INFO) << "14 downLinksList[0].downLanesList[0].linkWidth : " << v2xData.egoMap.downLinksList[0].downLanesList[0].linkWidth;
    LOG(INFO) << "15 downLinksList[0].PhaseID : " << (int)v2xData.egoMap.downLinksList[0].PhaseID;
    LOG(INFO) << "16 downLinksList[0].Azimuth : " << (int)v2xData.egoMap.downLinksList[0].Azimuth;
    LOG(INFO) << "17 adjacentLane[0].laneID : " << (int)v2xData.egoMap.adjacentLane[0].laneID;
    LOG(INFO) << "18 adjacentLane[0].laneWidth : " << v2xData.egoMap.adjacentLane[0].laneWidth;
    LOG(INFO) << "19 adjacentLane[0].maneuvers : " << v2xData.egoMap.adjacentLane[0].maneuvers;
    LOG(INFO) << "11 downLinksList[1].id : " << (int)v2xData.egoMap.downLinksList[1].id;
    LOG(INFO) << "12 downLinksList[1].linkWidth : " << v2xData.egoMap.downLinksList[1].linkWidth;
    LOG(INFO) << "13 downLinksList[1].downLanesList[0].id : " << (int)v2xData.egoMap.downLinksList[1].downLanesList[1].id;
    LOG(INFO) << "14 downLinksList[1].downLanesList[0].linkWidth : " << v2xData.egoMap.downLinksList[1].downLanesList[1].linkWidth;
    LOG(INFO) << "15 downLinksList[1].PhaseID : " << (int)v2xData.egoMap.downLinksList[1].PhaseID;
    LOG(INFO) << "16 downLinksList[1].Azimuth : " << (int)v2xData.egoMap.downLinksList[1].Azimuth;
    LOG(INFO) << "17 adjacentLane[1].laneID : " << (int)v2xData.egoMap.adjacentLane[1].laneID;
    LOG(INFO) << "18 adjacentLane[1].laneWidth : " << v2xData.egoMap.adjacentLane[1].laneWidth;
    LOG(INFO) << "19 adjacentLane[1].maneuvers : " << v2xData.egoMap.adjacentLane[1].maneuvers;
    LOG(INFO) << "------------------------------------------------------------------ " ;
}

void V2xFusionAlgo::ProcessObjVehiMapInfo(uint8_t* buf, uint32_t len)
{
    if (len != sizeof(V2X::ObjVehMapInfo) * ADAS_OBJ_VEH_INFO_NUM)
    {
        LOG(INFO) << "--------------------ProcessObjVehiMapInfo------------------- " ;
        LOG(INFO) << "len != sizeof(V2X::ObjVehMapInfo) * ADAS_OBJ_VEH_INFO_NUM " ;
        LOG(INFO) << "len  " << len;
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.objMap, buf, len);
    for(int i= 0; i < ADAS_OBJ_VEH_INFO_NUM; i++)
    {
        LOG(INFO) << "rev[" << i <<"]--------------------ProcessObjVehiMapInfo------------------- " ;
        LOG(INFO) << "1 belongsLink.upstreamNodeId : " << v2xData.objMap[i].belongsLink.upstreamNodeId;
        LOG(INFO) << "2 belongsLink.downNodeId : " << v2xData.objMap[i].belongsLink.downNodeId;
        LOG(INFO) << "3 belongsLink.laneID : " << (int)v2xData.objMap[i].belongsLink.laneID;
        LOG(INFO) << "4 localId : " << (int)v2xData.objMap[i].localId;
        LOG(INFO) << "------------------------------------------------------------------ " ;
    }
}

void V2xFusionAlgo::ProcessGSentryWarningInfo(uint8_t* buf, uint32_t len)
{
    if (len != sizeof(V2X::WarningInfo) * 3)
    {
        LOG(INFO) << "--------------------ProcessGSentryWarningInfo------------------- " ;
        LOG(INFO) << "len != sizeof(V2X::WarningInfo)  return " ;
        LOG(INFO) << "len  " << len;
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    memcpy(&v2xData.warningInfo, buf, len);
    for(int i= 0; i < 3; i ++)
    {
        LOG(INFO) << "v2vwarning[" << i << "]--------------------ProcessGSentryWarningInfo------------------- " ;
        LOG(INFO) << "1 warningInfo[0].warningType : " << (int)v2xData.warningInfo[i].warningType;
        LOG(INFO) << "2 warningInfo[0].level : " << (int)v2xData.warningInfo[i].level;
        LOG(INFO) << "3 warningInfo[0].remoteLocalId : " << v2xData.warningInfo[i].remoteLocalId;
        LOG(INFO) << "4 warningInfo[0].remoteBsmId : " << v2xData.warningInfo[i].remoteBsmId;
        LOG(INFO) << "5 warningInfo[0].objectCollisionTTC : " << v2xData.warningInfo[i].objectCollisionTTC;
        LOG(INFO) << "------------------------------------------------------------------ " ;
    }
    TransV2xWarn2CddWarn(v2xData.warningInfo[0], fusion.gSentryWarningInfo);
    WirteBack(reinterpret_cast<char*>(&fusion.gSentryWarningInfo), static_cast<uint16_t>(sizeof(CDDFusion::CDDgSentryWarningInfo)));
    
	CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_GSENTRY_WARN, (const char *)(&fusion.gSentryWarningInfo), sizeof(fusion.gSentryWarningInfo)});
}


/* coordinate system transformation algorithm */

void V2xFusionAlgo::TransV2xVehi2CddVehi(const V2X::AdasObjVehInfo& raw, const CAN::HostVehiclePos& host, CDD_Fusion_ObjInfo_BUS& dest)
{
    /* 计算过程中，朝向角以camera为准，即车辆行驶方向与正北方向的夹角，逆时针为正 */
    dest.De_ID_u8 = raw.localId;
    dest.De_Timestamp_u32 = raw.timeStamp;
    dest.De_measurement_status_u8 = 1;
    dest.De_length_f32 = raw.size.length;
    dest.De_width_f32 = raw.size.width;
    dest.De_height_f32 = raw.size.height;
    dest.De_Yaw_f32 = -raw.objectHeadingAngle * 0.0125 * PI / PI_DEGREE;
    vector<float>&& displacement = TransWgs84ToVcsCoordinate(host, raw.vehicelPos);
    dest.De_dx_f32 = displacement[0];
    dest.De_dy_f32 = displacement[1];

    const double hostHeadRad  = -(host.objectHeadingAngle * CAN_HEAD_ANGLE_DIAMETER - CAN_HEAD_ANGLE_OFFSET) * PI / PI_DEGREE;
    vector<float>&& velocity = TransObjVcsToHostVcs(raw.speed * V2X_VELOCITY_DIAMETER, 0, hostHeadRad, dest.De_Yaw_f32);
    dest.De_vx_f32 = velocity[0];
    dest.De_vy_f32 = velocity[1];

    vector<float>&& acceleration = TransObjVcsToHostVcs(raw.accelSet.longitude * V2X_ACCELERATION_DIAMETER, 
                                    raw.accelSet.latitude * V2X_ACCELERATION_DIAMETER, hostHeadRad, dest.De_Yaw_f32);
    dest.De_ax_f32 = acceleration[0];
    dest.De_ay_f32 = acceleration[1];
    dest.De_source_u8 = 1;
    /* 未填充数据 */
    // dest.conf;           gSentry无法获取该信息
    // dest.lifeTime;       gSentry无法获取该信息
    // dest.yawConf;        gSentry无法获取该信息
    // dest.yawRate;        gSentry无法获取该信息
    // dest.dxVariance;     gSentry无法获取该信息
    // dest.dyVariance;     gSentry无法获取该信息
    // dest.vxVariance;     gSentry无法获取该信息
    // dest.vyVariance;     gSentry无法获取该信息
    // dest.axVariance;     gSentry无法获取该信息
    // dest.ayVariance;     gSentry无法获取该信息
    // dest.currLane;       该信息不在目标车辆信息中
    // dest.ttc;            该信息不在目标车辆信息中
    // dest.ettc;           gSentry无法获取该信息
    // dest.cipv;           gSentry无法获取该信息
}

void V2xFusionAlgo::TransV2xSpat2CddSpat(const V2X::AdasSpatInfo& v2x, CDDFusion::CDDCurntLaneTrafficLightInfo& cdd)
{
    /* CDD灯色时间，解读为目标灯色开始时间 */
    cdd.trafficLightSt = v2x.spatInfoValid;
    uint16_t nextStateTime = v2x.curCoutingTime;
    uint16_t nnextStateTime = v2x.curCoutingTime + v2x.nextDurationTime;
    const uint16_t YELLOW_GATE = 7;
    const uint16_t GREEN_GATE = 4;
    const uint16_t RED_GATE = 2;

    if (v2x.lightState >= YELLOW_GATE)
    {
        cdd.yellowTime = 0;
        cdd.redTime = nextStateTime;
        cdd.greenTime = nnextStateTime;
    }
    else if (v2x.lightState >= GREEN_GATE)
    {
        cdd.greenTime = 0;
        cdd.yellowTime = nextStateTime;
        cdd.redTime = nnextStateTime;
    }
    else if (v2x.lightState >= RED_GATE)
    {
        cdd.redTime = 0;
        cdd.greenTime = nextStateTime;
        cdd.yellowTime = nnextStateTime;
    }
    else
    {
        cdd.trafficLightSt = 0;
        cdd.redTime = 0xFFFF;
        cdd.greenTime = 0xFFFF;
        cdd.yellowTime = 0xFFFF;
    }

    if (cdd.trafficLightSt == 0)
    {
        LOG(INFO) << "trafficLightSt : error";
        return ;
    }

    LOG(INFO) << "redTime : " << cdd.redTime;
    LOG(INFO) << "greenTime : " << cdd.greenTime;
    LOG(INFO) << "yellowTime : " << cdd.yellowTime;

}

void V2xFusionAlgo::TransV2xWarn2CddWarn(const V2X::WarningInfo& v2x, CDDFusion::CDDgSentryWarningInfo& cdd)
{
    cdd.warningType = v2x.warningType;
    cdd.level = v2x.level;
    cdd.targetID = v2x.remoteLocalId;
}

vector<float> V2xFusionAlgo::TransWgs84ToVcsCoordinate(const CAN::HostVehiclePos& host, const V2X::Position& remote)
{
    /* input parameter preprocess */
    /* input parameter is in CAN/V2X protocol, should transfer 0.0125° to rad and so on */
    const double hostLonRad = (host.longitude *  V2X_CAN_DEGREE_DIAMETER - CAN_DEGREE_OFFSET) * PI / PI_DEGREE;
    const double hostLatRad = (host.latitude *  V2X_CAN_DEGREE_DIAMETER - CAN_DEGREE_OFFSET) * PI / PI_DEGREE;
    const double hostElev   = host.elevation * CAN_ELEVATION_DIAMETER - CAN_ELEVATION_OFFSET;
    const double hostHeadRad  = -(host.objectHeadingAngle * CAN_HEAD_ANGLE_DIAMETER - CAN_HEAD_ANGLE_OFFSET) * PI / PI_DEGREE;
    const double remoteLonRad = remote.longitude *  V2X_CAN_DEGREE_DIAMETER * PI / PI_DEGREE;
    const double remoteLatRad = remote.latitude *  V2X_CAN_DEGREE_DIAMETER * PI / PI_DEGREE;
    const double remoteElev   = remote.elevation * V2X_ELEVATION_DIAMETER;

    /* Wgs84 ==============> ECEF*/
    const double hostCurveRadius = MAJOR_AXIS / sqrt(1 - ee * sin(hostLatRad) * sin(hostLatRad));
    const double remoteCurveRadius = MAJOR_AXIS / sqrt(1 - ee * sin(remoteLatRad) * sin(remoteLatRad));

    const double hostEcefX = (hostCurveRadius + hostElev) * cos(hostLatRad) * cos(hostLonRad);
    const double hostEcefY = (hostCurveRadius + hostElev) * cos(hostLatRad) * sin(hostLonRad);
    const double hostEcefZ = (hostCurveRadius * (1 - ee) + hostElev) * sin(hostLatRad);

    const double remoteEcefX = (remoteCurveRadius + remoteElev) * cos(remoteLatRad) * cos(remoteLonRad);
    const double remoteEcefY = (remoteCurveRadius + remoteElev) * cos(remoteLatRad) * sin(remoteLonRad);
    const double remoteEcefZ = (remoteCurveRadius * (1 - ee) + remoteElev) * sin(remoteLatRad);

    /* ECEF ==============> ENU */

    Eigen::Vector3d deltaPos(remoteEcefX - hostEcefX, remoteEcefY - hostEcefY, remoteEcefZ - hostEcefZ);
    Eigen::Matrix3d enuTransMatrix {
        {-sin(hostLonRad),                    cos(hostLonRad),                          0       },
        {-sin(hostLatRad) * cos(hostLonRad), -sin(hostLatRad) * sin(hostLonRad), cos(hostLatRad)},
        {cos(hostLatRad) * cos(hostLonRad),  cos(hostLatRad) * sin(hostLonRad), sin(hostLatRad) }
    };
    Eigen::Vector3d enupos = enuTransMatrix * deltaPos;

    /* ENU ==============> VCS */

    Eigen::Matrix3d vcsTransMatrix  {
        { cos(hostHeadRad + 0.5 * PI), sin(hostHeadRad + 0.5 * PI), 0}, 
        {-sin(hostHeadRad + 0.5 * PI), cos(hostHeadRad + 0.5 * PI), 0},
        {       0        ,         0        , 1}
    };
    Eigen::Vector3d ans = vcsTransMatrix * enupos;

    std::vector<float> ansvec(ans.data(), ans.data() + ans.rows() * ans.cols());

    return std::move(ansvec);
}

vector<float> V2xFusionAlgo::TransObjVcsToHostVcs(const uint16_t objXVector, const uint16_t objYVector, const float hosthead, const float remotehead)
{
    vector<uint16_t> velocity{objXVector, objYVector};
    const float rotateAngle = hosthead - remotehead;
    vector<vector<double>> transMatrix = {{ cos(rotateAngle), sin(rotateAngle)},
                                          {-sin(rotateAngle), cos(rotateAngle)}};
    vector<float> ans;
    for (uint32_t i=0; i<transMatrix.size(); i++)
    {
        double tmpans = 0;
        for (uint32_t j=0; j<transMatrix[0].size(); j++)
        {
            tmpans += transMatrix[i][j] * velocity[j];
        }
        ans.push_back(tmpans);
    }
    return std::move(ans);
}

void V2xFusionAlgo::WirteBack(char* buf, uint16_t len)
{
#ifdef __x86_64__
    std::shared_ptr<IProxy> ptr;
    if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::V2X, ptr))
    {
        std::shared_ptr<GSentryProxy> gSentryProxy = std::dynamic_pointer_cast<GSentryProxy>(ptr);
        gSentryProxy->Write(IProxy::ConnectType::TCP_CLIENT, buf, len);
        // CDebugFun::PrintBuf(reinterpret_cast<uint8_t*>(buf), len);
    }
#endif
}
