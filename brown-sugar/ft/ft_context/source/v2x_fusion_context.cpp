#include "v2x_fusion_context.h"

void V2xDataFusionTest::SetUp()
{
    CAN::HostVehiclePos host;
    host.latitude = 2099985554;
    host.longitude = 2940016624;
    host.elevation = 40000;
    host.objectHeadingAngle = 0;
    host.isHostPosValid = true;
    MockHostVehiInfo(host);
    V2X::GSentryStatus status = {3,false};
    MockGSentryStatus(status);
}

void V2xDataFusionTest::TearDown()
{
    DataRepo::GetInstance().Clear();
    memset(buff,0,sizeof(buff));
}

void V2xDataFusionTest::MockHostVehiInfo(const CAN::HostVehiclePos& host)
{
    EventMessage msg = {MsgType::CAN, reinterpret_cast<const char*>(&host), sizeof(host)};
    EventDispatcher::ProcessMessage(msg);
    const CAN::HostVehiclePos& cmpHost = DataRepo::GetInstance().GetHostVehicle();
    ASSERT_EQ(host.elevation, cmpHost.elevation);
    ASSERT_EQ(host.latitude, cmpHost.latitude);
    ASSERT_EQ(host.longitude, cmpHost.longitude);
    ASSERT_EQ(host.objectHeadingAngle, cmpHost.objectHeadingAngle);
}

void V2xDataFusionTest::MockGSentryStatus(V2X::GSentryStatus& status)
{
    uint16_t len = MockV2xHead(EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT, sizeof(status));

    memcpy(&buff[len], &status, sizeof(status));
    EventMessage msg = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), static_cast<uint16_t>(len + sizeof(status))};
    EventDispatcher::ProcessMessage(msg);
    const V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    ASSERT_EQ(status.faultStatus, v2xData.status.faultStatus);
    ASSERT_EQ(status.gSentryStatus, v2xData.status.gSentryStatus);
}

uint16_t V2xDataFusionTest::MockV2xHead(uint16_t msgId, uint32_t msgLen)
{
    V2X::V2xAdasMsgHeader* head = new(&buff[0]) V2X::V2xAdasMsgHeader();
    head->msgId = msgId;
    head->msgLen = msgLen;
    return sizeof(V2X::V2xAdasMsgHeader);
}

void V2xDataFusionTest::MockV2xSpatInfo(V2X::AdasSpatInfo spat[], uint16_t num)
{
    uint16_t payloadLen = sizeof(V2X::AdasSpatInfo) * num;
    uint16_t len = MockV2xHead(EV_GSENTRY_ADAS_SPATINFO_REPORT, payloadLen);
    memcpy(&buff[len], &spat[0], payloadLen);

    EventMessage msg = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), static_cast<uint16_t>(len + payloadLen)};
    EventDispatcher::ProcessMessage(msg);
}

void V2xDataFusionTest::MockV2xExtraMap(const V2X::MapAddResult& extarMap)
{
    uint16_t len = MockV2xHead(EV_GSENTRY_ADAS_CALC_MAPINFO_REPORT, sizeof(extarMap));

    memcpy(&buff[len], &extarMap, sizeof(extarMap));
    EventMessage msg = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), static_cast<uint16_t>(len + sizeof(extarMap))};
    EventDispatcher::ProcessMessage(msg);
    const V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    ASSERT_EQ(extarMap.distToNode, v2xData.mapAddRes.distToNode);
    ASSERT_EQ(extarMap.isAtAcross, v2xData.mapAddRes.isAtAcross);
    ASSERT_EQ(extarMap.offsetTolane, v2xData.mapAddRes.offsetTolane);
    ASSERT_EQ(extarMap.offsetTolink, v2xData.mapAddRes.offsetTolink);
}

void V2xDataFusionTest::MockV2xHostVehiMap(const V2X::EgoVehMapInfo& hostMap)
{
    uint16_t len = MockV2xHead(EV_GSENTRY_ADAS_EGOVEHI_MAPINFO_REPORT, sizeof(hostMap));

    memcpy(&buff[len], &hostMap, sizeof(hostMap));
    EventMessage msg = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), static_cast<uint16_t>(len + sizeof(hostMap))};
    EventDispatcher::ProcessMessage(msg);
    const V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    ASSERT_EQ(memcmp(&hostMap, &v2xData.egoMap, sizeof(hostMap)) , 0);
}

void V2xDataFusionTest::MockV2xObjVehiMap(V2X::ObjVehMapInfo objMap[], uint16_t num)
{
    uint16_t payloadLen = sizeof(V2X::ObjVehMapInfo) * num;
    uint16_t len = MockV2xHead(EV_GSENTRY_ADAS_OBJVEHI_MAPINFO_REPORT, payloadLen);
    memcpy(&buff[len], &objMap[0], payloadLen);

    EventMessage msg = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), static_cast<uint16_t>(len + payloadLen)};
    EventDispatcher::ProcessMessage(msg);
    const V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
    ASSERT_EQ(memcmp(&objMap[0], &v2xData.objMap[0], payloadLen) , 0);
}

void V2xDataFusionTest::MockV2xgSentryWarn(const V2X::WarningInfo& gSentryWarn)
{
    uint16_t len = MockV2xHead(EV_GSENTRY_ADAS_WARNING_REPORT, sizeof(gSentryWarn));
    memcpy(&buff[len], &gSentryWarn, sizeof(gSentryWarn));

    EventMessage msg = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), static_cast<uint16_t>(len + sizeof(gSentryWarn))};
    EventDispatcher::ProcessMessage(msg);
}

void V2xDataFusionTest::MockV2xObjVehiInfo(V2X::AdasObjVehInfo objVehi[], uint16_t num)
{
    uint16_t payloadLen = sizeof(V2X::AdasObjVehInfo) * num;
    uint16_t len = MockV2xHead(EV_GSENTRY_ADAS_OBJECT_VEHICLE_REPORT, payloadLen);
    memcpy(&buff[len], &objVehi[0], payloadLen);

    EventMessage msg = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), static_cast<uint16_t>(len + payloadLen)};
    EventDispatcher::ProcessMessage(msg);
}

