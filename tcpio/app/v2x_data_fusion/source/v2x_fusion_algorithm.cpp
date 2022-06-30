#include "v2x_fusion_algorithm.h"
#include <cmath>

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

void V2xFusionAlgo::TransV2xVehi2CddVehi(const V2X::AdasObjVehInfo& raw, const CAN::HostVehiclePos& host, CDDFusion::CDDFusionGSentryObj& dest)
{
    dest.id = raw.localId;
    dest.timestamp = raw.timeStamp;
    dest.measurementStatus = 1;
    dest.length = raw.size.length;
    dest.width = raw.size.width;
    dest.height = raw.size.height;
    dest.yaw = -raw.objectHeadingAngle * 0.0125 * PI / PI_DEGREE;
    /*gSentry定义：目标车辆航向角为运动方向与正北方向的顺时针夹角, 取值0~28800，单位0.0125°
    J3定义：     目标的方位角(朝向)，方向和vcs坐标系一致，左正右负，即逆时针正，顺时针负 */
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
    // dest.ttc = raw.;     ttc不在这个结构体内
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
    /* 计算过程中，朝向角以camera为准，即车辆行驶方向与正北方向的夹角，逆时针为正 */
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

