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
#include <iomanip>

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
    const double V2X_ANGLE_DIEMETER = 0.0125;
    const double V2X_OBJECT_NON_STATION_BASE = 0.001;
    const double OBJECT_MOVING_SPEED_GATE = 2.0;
    const double OBJECT_STATION_SPEED_GATE = 0.0;
    const int PI_DEGREE = 180;
    const double MAJOR_AXIS = 6378137.0;
    const double MINOR_AXIS = 6356752.3142;
    const double ee = (MAJOR_AXIS + MINOR_AXIS) * (MAJOR_AXIS - MINOR_AXIS) / (MAJOR_AXIS * MAJOR_AXIS);
}

void V2xFusionAlgo::ProcessRecieveData(uint8_t* data, uint16_t len)
{
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
        case EV_GSENTRY_ADAS_ROADPART_REPORT:           ProcessRoadPartInfo(payload, length); break;
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
    LOG(INFO) << "--------------------ProcessGSentrySatatus------------------- " 
    << "\n\t1 gSentryStatus : " << int(gSentryStatus.gSentryStatus)
    << "\n\t2 faultStatus : " << int(gSentryStatus.faultStatus)
    << "\n------------------------------------------------------------ " ;
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
        LOG(INFO) << "\tlen != sizeof(V2X::MapAddResult) return " ;
        LOG(INFO) << "\tlen  " << len;
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.mapAddRes, buf, len);
    LOG(INFO) << "--------------------ProcessHostVehiExtraMapInfo------------------- " 
    << "\n\t1 offsetTolink : " << v2xData.mapAddRes.offsetTolink
    << "\n\t2 offsetTolane : " << v2xData.mapAddRes.offsetTolane
    << "\n\t3 distToNode : " << v2xData.mapAddRes.distToNode
    << "\n\t4 isAtAcross : " << (int)v2xData.mapAddRes.isAtAcross
    << "\n------------------------------------------------------------------ " ;
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
    for(int i = 0; i<ADAS_SPAT_INFO_NUM;i++)
    {
    LOG(INFO) << "--------------------ProcessSpatInfo------------------- " 
    << "\n\t1 spatInfo["<< i<< "].timeStamp : " << v2xData.spatInfo[i].timeStamp
    << "\n\t2 spatInfo["<< i<< "].spatInfoValid : " << (int)v2xData.spatInfo[i].spatInfoValid
    << "\n\t3 spatInfo["<< i<< "].belongsNodeId : " << v2xData.spatInfo[i].belongsNodeId
    << "\n\t4 spatInfo["<< i<< "].phaseID : " << v2xData.spatInfo[i].phaseID
    << "\n\t5 spatInfo["<< i<< "].curCoutingTime : " << v2xData.spatInfo[i].curCoutingTime
    << "\n\t6 spatInfo["<< i<< "].nextLight : " << (int)v2xData.spatInfo[i].nextLight
    << "\n\t7 spatInfo["<< i<< "].nextDurationTime : " << v2xData.spatInfo[i].nextDurationTime
    << "\n------------------------------------------------------------------ " ;
    }
    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    memset(&fusion.spatInfo, 0, sizeof(fusion.spatInfo));
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
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.objVehicle[0], buf, len);
    for(int i = 0; i < ADAS_OBJ_VEH_INFO_NUM; i++)
    {
        if ((int)v2xData.objVehicle[i].localId > 0) {
            LOG(INFO) << "reveh[" << i << "]--------------------ProcessObjVehiInfo------------------- " 
            << "\n\t1 timeStamp : " << v2xData.objVehicle[i].timeStamp
            << "\n\t2 localId : " << (int)v2xData.objVehicle[i].localId
            << "\n\t3 objectSource : " << (int)v2xData.objVehicle[i].objectSource
            << "\n\t4 vehicleClass : " << (int)v2xData.objVehicle[i].vehicleClass
            << "\n\t5 length : " << v2xData.objVehicle[i].size.length
            << "\n\t6 width : " << v2xData.objVehicle[i].size.width
            << "\n\t7 height : " << (int)v2xData.objVehicle[i].size.height
            << "\n\t8 objectHeadingAngle : " << v2xData.objVehicle[i].objectHeadingAngle
            << "\n\t9 objectYawAngle : " << v2xData.objVehicle[i].objectYawAngle
            << "\n\t10 gear : " << (int)v2xData.objVehicle[i].gear
            << "\n\t11 steeringWheelAngle : " << (int)v2xData.objVehicle[i].steeringWheelAngle
            << "\n\t12 remoteLight : " << (int)v2xData.objVehicle[i].remoteLight
            << "\n\t13 brakePedalStatus : " << (int)v2xData.objVehicle[i].vehicleBrakes.brakePedalStatus
            << "\n\t14 ABS : " << (int)v2xData.objVehicle[i].vehicleBrakes.absBrakes
            << "\n\t15 brakeAppliedStatus : " << (int)v2xData.objVehicle[i].vehicleBrakes.brakeAppliedStatus
            << "\n\t16 brakeBoostApplied : " << (int)v2xData.objVehicle[i].vehicleBrakes.brakeBoostApplied
            << "\n\t17 auxiliaryBrakeStatus : " << (int)v2xData.objVehicle[i].vehicleBrakes.auxiliaryBrakeStatus
            << "\n\t18 stabilityControlStatus : " << (int)v2xData.objVehicle[i].stabilityControlStatus
            << "\n\t19 tractionControlStatus : " << (int)v2xData.objVehicle[i].tractionControlStatus
            << "\n\t21 espStatus : " << (int)v2xData.objVehicle[i].espStatus
            << "\n\t22 ldwStatus : " << (int)v2xData.objVehicle[i].ldwStatus
            << "\n\t23 speed : " << (int)v2xData.objVehicle[i].speed
            << "\n\t24 accelSet.longitude : " << (int)v2xData.objVehicle[i].accelSet.longitude
            << "\n\t25 accelSet.latitude : " << (int)v2xData.objVehicle[i].accelSet.latitude
            << "\n\t26 targetClassification : " << (int)v2xData.objVehicle[i].targetClassification
            << "\n\t27 mapInfo.offsetTolink : " << (int)v2xData.objVehicle[i].mapInfo.offsetTolink
            << "\n\t28 mapInfo.offsetTolane : " << (int)v2xData.objVehicle[i].mapInfo.offsetTolane
            << "\n\t29 mapInfo.distToNode : " << (int)v2xData.objVehicle[i].mapInfo.distToNode
            << "\n\t30 mapInfo.isAtAcross : " << (int)v2xData.objVehicle[i].mapInfo.isAtAcross
            << "\n\t31 mapInfo.linkRealtion : " << (int)v2xData.objVehicle[i].mapInfo.linkRealtion
            << "\n------------------------------------------------------------------ " ;
        }
    }

    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    memset(&fusion.cddObjects, 0, sizeof(fusion.cddObjects));

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


    LOG(INFO) << "--------------------ProcessHostVehiMapInfo------------------- " 
    << "\n\t1 curLink.upstreamNode.RegionId : " << v2xData.egoMap.curLink.upstreamNode.RegionId
    << "\n\t2 curLink.upstreamNode.id : " << v2xData.egoMap.curLink.upstreamNode.id
    << "\n\t3 curLink.downNode : " << v2xData.egoMap.curLink.downNode
    << "\n\t4 curLink.linkWidth : " << v2xData.egoMap.curLink.linkWidth
    << "\n\t5 curLink.speedLimit.minSpeed : " << v2xData.egoMap.curLink.speedLimit.minSpeed
    << "\n\t6 curLink.speedLimit.maxSpeed : " << v2xData.egoMap.curLink.speedLimit.maxSpeed
    << "\n\t7 curLane.laneID : " << (int)v2xData.egoMap.curLane.laneID
    << "\n\t8 curLane.laneWidth : " << v2xData.egoMap.curLane.laneWidth
    << "\n\t9 curLane.maneuvers : " << v2xData.egoMap.curLane.maneuvers
    << "\n\t10 curLane.lanePointNextNo : " << (int)v2xData.egoMap.curLane.lanePointNextNo;
    for (int i = 0; i < CONNECTION_NUM_PER_LANE; i++)
    {
        if (v2xData.egoMap.curLane.connectsToLaneIdList[i].laneID != 0)
        LOG(INFO) << "\n\t11 curLane.connectsToLaneIdList->belongLinkID[" << i << "].id : " << (int)v2xData.egoMap.curLane.connectsToLaneIdList[i].belongLinkID
            << "\n\t11 curLane.connectsToLaneIdList->laneID;[" << i << "].id : " << (int)v2xData.egoMap.curLane.connectsToLaneIdList[i].laneID;
    }
    for (int i = 0; i < CONNECTION_NUM_PER_LINK; i++)
    {
        if (v2xData.egoMap.downLinksList[i].linkWidth > 0)
        {
            LOG(INFO) << "\n\t11 downLinksList[" << i << "].id : " << (int)v2xData.egoMap.downLinksList[i].id
            << "\n\t12 downLinksList[" << i << "].linkWidth : " << v2xData.egoMap.downLinksList[i].linkWidth
            << "\n\t13 downLinksList[" << i << "].downLanesList[0].id : " << (int)v2xData.egoMap.downLinksList[i].downLanesList[0].id
            << "\n\t14 downLinksList[" << i << "].downLanesList[0].linkWidth : " << v2xData.egoMap.downLinksList[i].downLanesList[0].linkWidth
            << "\n\t15 downLinksList[" << i << "].PhaseID : " << (int)v2xData.egoMap.downLinksList[i].PhaseID
            << "\n\t16 downLinksList[" << i << "].Azimuth : " << (int)v2xData.egoMap.downLinksList[i].Azimuth;
        }
    }
    for(int i = 0; i < ADAS_ADJACENT_LANE_NUM; i++)
    {
        if (v2xData.egoMap.adjacentLane[i].laneWidth > 0)
        {
            LOG(INFO) << "\n\t17 adjacentLane[" << i << "].laneID : " << (int)v2xData.egoMap.adjacentLane[i].laneID
            << "\n\t18 adjacentLane[" << i << "].laneWidth : " << v2xData.egoMap.adjacentLane[i].laneWidth
            << "\n\t19 adjacentLane[" << i << "].maneuvers : " << v2xData.egoMap.adjacentLane[i].maneuvers;
        }
    }
    LOG(INFO) << "\n------------------------------------------------------------------ " ;
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
        LOG(INFO) << "rev[" << i <<"]--------------------ProcessObjVehiMapInfo------------------- " 
        << "\n\t1 belongsLink.upstreamNodeId : " << v2xData.objMap[i].belongsLink.upstreamNodeId
        << "\n\t2 belongsLink.downNodeId : " << v2xData.objMap[i].belongsLink.downNodeId
        << "\n\t3 belongsLink.laneID : " << (int)v2xData.objMap[i].belongsLink.laneID
        << "\n\t4 localId : " << (int)v2xData.objMap[i].localId
        << "\n------------------------------------------------------------------ " ;
    }
}

void V2xFusionAlgo::ProcessGSentryWarningInfo(uint8_t* buf, uint32_t len)
{
    if (len != sizeof(V2X::WarningInfo) * ADAS_WARN_INFO_NUM)
    {
        LOG(INFO) << "--------------------ProcessGSentryWarningInfo------------------- " ;
        LOG(INFO) << "len != sizeof(V2X::WarningInfo)  return " ;
        LOG(INFO) << "len  " << len;
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    memcpy(&v2xData.warningInfo, buf, len);
    for(int i= 0; i < ADAS_WARN_INFO_NUM; i ++)
    {
        LOG(INFO) << "v2vwarning[" << i << "]--------------------ProcessGSentryWarningInfo------------------- " 
        << "\n\t1 warningInfo[" << i <<"].warningType : " << (int)v2xData.warningInfo[i].warningType
        << "\n\t2 warningInfo[" << i << "].level : " << (int)v2xData.warningInfo[i].level
        << "\n\t3 warningInfo[" << i << "].remoteLocalId : " << v2xData.warningInfo[i].remoteLocalId
        << "\n\t4 warningInfo[" << i << "].remoteBsmId : " << v2xData.warningInfo[i].remoteBsmId
        << "\n\t5 warningInfo[" << i << "].objectCollisionTTC : " << v2xData.warningInfo[i].objectCollisionTTC
        << "\n------------------------------------------------------------------ " ;
    }
    TransV2xWarn2CddWarn(v2xData.warningInfo[0], fusion.gSentryWarningInfo);
    WirteBack(reinterpret_cast<char*>(&fusion.gSentryWarningInfo), static_cast<uint16_t>(sizeof(CDDFusion::CDDgSentryWarningInfo)));
    
	CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_GSENTRY_WARN, (const char *)(&fusion.gSentryWarningInfo), sizeof(fusion.gSentryWarningInfo)});
}

void V2xFusionAlgo::ProcessRoadPartInfo(uint8_t* buf, uint32_t len)
{
    if (len != sizeof(V2X::RoadPartInfo))
    {
        LOG(INFO) << "--------------------ProcessRoadPartInfo------------------- " ;
        LOG(INFO) << "len != sizeof(V2X::RoadPartInfo)  return " ;
        LOG(INFO) << "len  " << len;
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.roadPartInfo, buf, len);

    for(int i= 0; i < MAX_PARTCIPANT_NUM; i ++)
    {
        if (v2xData.roadPartInfo.participants[i].ptcType > 0)
        {
            LOG(INFO) << "participants[" << i << "]--------------------ProcessRoadPartInfo---------------- " ;
            LOG(INFO) << "1 participants.rsuId : " << v2xData.roadPartInfo.participants[i].rsuId;
            LOG(INFO) << "2 participants.ptcType : " << v2xData.roadPartInfo.participants[i].ptcType;
            LOG(INFO) << "3 participants.ptcId : " << v2xData.roadPartInfo.participants[i].ptcId;
            LOG(INFO) << "4 participants.source : " << v2xData.roadPartInfo.participants[i].source;
            LOG(INFO) << "5 participants.id " << v2xData.roadPartInfo.participants[i].id;
            LOG(INFO) << "6 participants.timestamp_ms : " << (uint64_t)v2xData.roadPartInfo.participants[i].timestamp_ms;
            LOG(INFO) << "7 participants.speed_ms : " << v2xData.roadPartInfo.participants[i].speed_ms;
            LOG(INFO) << "8 participants.heading : " << v2xData.roadPartInfo.participants[i].heading;
            LOG(INFO) << "9 participants.steeringWheelAngle : " << v2xData.roadPartInfo.participants[i].steeringWheelAngle;
            LOG(INFO) << "10 participants.gear " << v2xData.roadPartInfo.participants[i].gear;
            LOG(INFO) << "------------------------------------------------------------------ " ;
        }
    }
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
    dest.De_Yaw_f32 = -raw.objectHeadingAngle * V2X_ANGLE_DIEMETER * PI / PI_DEGREE;

    vector<float>&& displacement = TransWgs84ToVcsCoordinate(host, raw.vehicelPos);
    dest.De_dx_f32 = displacement[0];
    dest.De_dy_f32 = displacement[1];

    const double hostHeadRad  = -host.objectHeadingAngle * PI / PI_DEGREE;
    vector<float>&& velocity = TransObjVcsToHostVcs(raw.speed * V2X_VELOCITY_DIAMETER, 0, hostHeadRad, dest.De_Yaw_f32);
    dest.De_vx_f32 = velocity[0] - host.speed;
    dest.De_vy_f32 = velocity[1] + V2X_OBJECT_NON_STATION_BASE;

    vector<float>&& acceleration = TransObjVcsToHostVcs(raw.accelSet.longitude * V2X_ACCELERATION_DIAMETER, 
                                    raw.accelSet.latitude * V2X_ACCELERATION_DIAMETER, hostHeadRad, dest.De_Yaw_f32);
    dest.De_ax_f32 = acceleration[0] - host.acc_x;
    dest.De_ay_f32 = acceleration[1] - host.acc_y;
    dest.De_source_u8 = static_cast<uint8_t>(OBU_V2X_SOURCE);
    dest.De_life_time_u32 = 0xFFFF;
    dest.De_ObjectType_u8 = static_cast<uint8_t>(ObstacleType_VehicleFull);

    // Moving到Moving_Slowly：7.2km/h
    float obj_velocity = raw.speed * V2X_VELOCITY_DIAMETER;
    if (obj_velocity >= OBJECT_MOVING_SPEED_GATE)
    {
        // moving
        dest.De_ObjectMovingStatus_u8 = static_cast<uint8_t>(MS_MOVING);
    }
    else if (obj_velocity > OBJECT_STATION_SPEED_GATE)
    {
        // moving slowly
        dest.De_ObjectMovingStatus_u8 = static_cast<uint8_t>(MS_MOVING_SLOWLY);
    }
    else
    {
        // stationary
        dest.De_ObjectMovingStatus_u8 = static_cast<uint8_t>(MS_STATIONARY);
    }
    dest.De_conf_f32 = 1.0;
    dest.De_dxVariance_f32 = 0;
    dest.De_dyVariance_f32 = 0;
    dest.De_vxVariance_f32 = 0;
    dest.De_vyVariance_f32 = 0;
    dest.De_axVariance_f32 = 0;
    dest.De_ayVariance_f32 = 0;

    LOG(INFO) << "host gnss value, long: " << std::fixed << std::setprecision(7) << host.longitude << "; lati: " 
              << std::fixed << std::setprecision(7) << host.latitude << "; height: " << std::fixed << std::setprecision(7) << host.elevation
              << "\n\tremote gnss value, long: " << raw.vehicelPos.longitude << "; lati: "<< raw.vehicelPos.latitude << "; height: " << raw.vehicelPos.elevation
              << "\n\thost accel value, long: " << host.acc_x << "; lati: "<< host.acc_y
              << "\n\tremote accel value, long: " << raw.accelSet.longitude << "; lati: "<< raw.accelSet.latitude
              << "\n\tremote heading angle: " << raw.objectHeadingAngle << ";  host heading angle: " << host.objectHeadingAngle 
              << "\n\tremote speed: " << raw.speed << "; host speed: " << host.speed
              << "\n\tcalc result. remote in vcs, dx: " << dest.De_dx_f32 <<  ",  dy: "<< dest.De_dy_f32;
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

    LOG(INFO) << "redTime : " << cdd.redTime << "\tgreenTime : " << cdd.greenTime << "\tyellowTime : " << cdd.yellowTime;

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
    const double hostLonRad = host.longitude * PI / PI_DEGREE;
    const double hostLatRad = host.latitude * PI / PI_DEGREE;
    const double hostElev   = host.elevation;
    const double hostHeadRad  = -host.objectHeadingAngle * PI / PI_DEGREE;
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

    CalibrateStruct& CalibrateVariable = DataRepo::GetInstance().GetCalibrateVariables();
    ans(0) += CalibrateVariable.dx_gnss_to_rear_center;
    ans(1) += CalibrateVariable.dy_gnss_to_rear_center;

    std::vector<float> ansvec(ans.data(), ans.data() + ans.rows() * ans.cols());

    return ansvec;
}

vector<float> V2xFusionAlgo::TransObjVcsToHostVcs(const float objXVector, const float objYVector, const float hosthead, const float remotehead)
{
    vector<float> velocity{objXVector, objYVector};
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
    return ans;
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
