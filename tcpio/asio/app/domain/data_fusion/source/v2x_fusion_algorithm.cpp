#include "v2x_fusion_algorithm.h"
#include "v2x_adas_event_macro.h"
#include <cmath>
#include <cstring>

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
    if (len < sizeof(V2X::V2xAdasMsgHeader))
    {
        return;
    }
    const V2X::V2xAdasMsgHeader& head = *reinterpret_cast<V2X::V2xAdasMsgHeader*>(data);
    uint8_t* payload = data + sizeof(V2X::V2xAdasMsgHeader);
    uint32_t length  = head.msgLen;
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
    if (buf == nullptr || len != sizeof(V2X::GSentryStatus))
    {
        return ;
    }
    const V2X::GSentryStatus gSentryStatus = *reinterpret_cast<V2X::GSentryStatus*>(buf);
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    v2xData.status.faultStatus = gSentryStatus.faultStatus;
    v2xData.status.gSentryStatus = gSentryStatus.gSentryStatus;
}

void V2xFusionAlgo::ProcessHostVehiExtraMapInfo(uint8_t* buf, uint32_t len)
{
    if (buf == nullptr || len != sizeof(V2X::MapAddResult))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.mapAddRes, buf, len);
    // printf("recieve gSentry Calc Map msg!\n");
}

void V2xFusionAlgo::ProcessSpatInfo(uint8_t* buf, uint32_t len)
{
    if (buf == nullptr || (len != sizeof(V2X::AdasSpatInfo) * ADAS_SPAT_INFO_NUM))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.spatInfo[0], buf, len);
    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    // 融合数据只用了当前车道下一个红绿灯信息，因此只取v2x数据的第一个红绿灯信息
    TransV2xSpat2CddSpat(v2xData.spatInfo[0], fusion.spatInfo);
    // printf("recieve gSentry Spat msg!\n");
}

void V2xFusionAlgo::ProcessObjVehiInfo(uint8_t* buf, uint32_t len)
{
    if (buf == nullptr || (len != sizeof(V2X::ObjVehMapInfo) * ADAS_OBJ_VEH_INFO_NUM))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.objVehicle[0], buf, len);

    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    CAN::HostVehiclePos& host = DataRepo::GetInstance().GetHostVehicle();

    uint16_t i = 0;
    v2xData.objVehiNum = 0;
    while(v2xData.objVehicle[i].localId != 0)
    {
        TransV2xVehi2CddVehi(v2xData.objVehicle[i], host, fusion.v2xObjVehi[i]);
        // printf("recieve gSentry ObjVehiInfo msg!\n");
        v2xData.objVehiNum++;
        i++;
    }
}

void V2xFusionAlgo::ProcessHostVehiMapInfo(uint8_t* buf, uint32_t len)
{
    if (buf == nullptr || (len != sizeof(V2X::EgoVehMapInfo)))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.egoMap, buf, len);
}

void V2xFusionAlgo::ProcessObjVehiMapInfo(uint8_t* buf, uint32_t len)
{
    if (buf == nullptr || len != sizeof(V2X::ObjVehMapInfo))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    memcpy(&v2xData.objMap, buf, len);
}

void V2xFusionAlgo::ProcessGSentryWarningInfo(uint8_t* buf, uint32_t len)
{
    if (buf == nullptr || len != sizeof(V2X::WarningInfo))
    {
        return ;
    }
    V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    CDDFusion::CddFusionRepo& fusion = DataRepo::GetInstance().GetCddFusionData();
    memcpy(&v2xData.warningInfo, buf, len);
    TransV2xWarn2CddWarn(v2xData.warningInfo, fusion.gSentryWarningInfo);
}


/* coordinate system transformation algorithm */

void V2xFusionAlgo::TransV2xVehi2CddVehi(const V2X::AdasObjVehInfo& raw, const CAN::HostVehiclePos& host, CDDFusion::CDDFusionGSentryObj& dest)
{
    /* 计算过程中，朝向角以camera为准，即车辆行驶方向与正北方向的夹角，逆时针为正 */
    dest.id = raw.localId;
    dest.timestamp = raw.timeStamp;
    dest.measurementStatus = 1;
    dest.length = raw.size.length;
    dest.width = raw.size.width;
    dest.height = raw.size.height;
    dest.yaw = -raw.objectHeadingAngle * 0.0125 * PI / PI_DEGREE;
    vector<float>&& displacement = TransWgs84ToVcsCoordinate(host, raw.vehicelPos);
    dest.dx = displacement[0];
    dest.dy = displacement[1];

    const double hostHeadRad  = -(host.objectHeadingAngle * CAN_HEAD_ANGLE_DIAMETER - CAN_HEAD_ANGLE_OFFSET) * PI / PI_DEGREE;
    vector<float>&& velocity = TransObjVcsToHostVcs(raw.speed * V2X_VELOCITY_DIAMETER, 0, hostHeadRad, dest.yaw);
    dest.vx = velocity[0];
    dest.vy = velocity[1];

    vector<float>&& acceleration = TransObjVcsToHostVcs(raw.accelSet.longitude * V2X_ACCELERATION_DIAMETER, 
                                    raw.accelSet.latitude * V2X_ACCELERATION_DIAMETER, hostHeadRad, dest.yaw);
    dest.ax = acceleration[0];
    dest.ay = acceleration[1];
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
    cdd.trafficLightSt = v2x.spatInfoValid;
    // 红绿灯状态填充
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
    /* calculate the coordinate index of remote in ENU system which is located at host */
    vector<double> deltaPos = {remoteEcefX - hostEcefX, remoteEcefY - hostEcefY, remoteEcefZ - hostEcefZ};
    vector<vector<double>> enuTransMatrix = { {-sin(hostLonRad),                    cos(hostLonRad),                          0       },
                                              {-sin(hostLatRad) * cos(hostLonRad), -sin(hostLatRad) * sin(hostLonRad), cos(hostLatRad)},
                                              { cos(hostLatRad) * cos(hostLonRad),  cos(hostLatRad) * sin(hostLonRad), sin(hostLatRad)}};

    vector<float> enupos;
    for (uint32_t i=0; i<enuTransMatrix.size(); i++)
    {
        double tmpans = 0;
        for (uint32_t j=0; j<enuTransMatrix[0].size(); j++)
        {
            tmpans += enuTransMatrix[i][j] * deltaPos[j];
        }
        enupos.push_back(tmpans);
    }

    /* ENU ==============> VCS */
    vector<vector<double>> vcsTransMatrix = { { cos(hostHeadRad), sin(hostHeadRad), 0},
                                              {-sin(hostHeadRad), cos(hostHeadRad), 0},
                                              {        0        ,        0        , 1}};
    vector<float> ans;
    for (uint32_t i=0; i<vcsTransMatrix.size(); i++)
    {
        double tmpans = 0;
        for (uint32_t j=0; j<vcsTransMatrix[0].size(); j++)
        {
            tmpans += vcsTransMatrix[i][j] * enupos[j];
        }
        ans.push_back(tmpans);
    }

    return std::move(ans);
}

vector<float> V2xFusionAlgo::TransObjVcsToHostVcs(const uint16_t objXVector, const uint16_t objYVector, const float hosthead, const float remotehead)
{
    vector<uint16_t> velocity{objXVector, objYVector};
    const float rotateAngle = hosthead - remotehead;
    vector<vector<double>> transMatrix = {{cos(rotateAngle), -sin(rotateAngle)},
                                          {sin(rotateAngle),  cos(rotateAngle)}};
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

