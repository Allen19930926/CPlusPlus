#include "v2x_fusion_context.h"

TEST_F(V2xDataFusionTest, gSentryStatus_abnormal_test)
{
    V2X::GSentryStatus status = {4,true};
    MockGSentryStatus(status);
}

TEST_F(V2xDataFusionTest, gSentryStatus_normal_test)
{
    V2X::GSentryStatus status = {2,false};
    MockGSentryStatus(status);
}

TEST_F(V2xDataFusionTest, normal_spatinfo_with_fault_gSentryStatus_test)
{
    V2X::GSentryStatus status = {4,true};
    MockGSentryStatus(status);

    V2X::AdasSpatInfo spat[ADAS_SPAT_INFO_NUM];
    spat[0].spatInfoValid = true;
    spat[0].lightState    = 5;
    spat[0].curCoutingTime = 70;
    spat[0].nextDurationTime = 20;
    MockV2xSpatInfo(spat, ADAS_SPAT_INFO_NUM);

    const CDDFusion::CddFusionRepo& cddData = DataRepo::GetInstance().GetCddFusionData();
    ASSERT_TRUE(!cddData.spatInfo.trafficLightSt);
    ASSERT_NE(cddData.spatInfo.redTime, spat[0].curCoutingTime + spat[0].nextDurationTime);
    ASSERT_NE(cddData.spatInfo.yellowTime, spat[0].curCoutingTime);
}

TEST_F(V2xDataFusionTest, normal_spatinfo_with_current_green_light_test)
{

    V2X::AdasSpatInfo spat[ADAS_SPAT_INFO_NUM];
    spat[0].spatInfoValid = true;
    spat[0].lightState    = 5;
    spat[0].curCoutingTime = 70;
    spat[0].nextDurationTime = 20;
    MockV2xSpatInfo(spat, ADAS_SPAT_INFO_NUM);

    const CDDFusion::CddFusionRepo& cddData = DataRepo::GetInstance().GetCddFusionData();
    ASSERT_TRUE(cddData.spatInfo.trafficLightSt);
    ASSERT_EQ(cddData.spatInfo.greenTime, 0);
    ASSERT_EQ(cddData.spatInfo.yellowTime, spat[0].curCoutingTime);
    ASSERT_EQ(cddData.spatInfo.redTime, spat[0].curCoutingTime + spat[0].nextDurationTime);
}

TEST_F(V2xDataFusionTest, normal_spatinfo_with_current_red_light_test)
{
    V2X::AdasSpatInfo spat[ADAS_SPAT_INFO_NUM];
    spat[0].spatInfoValid = true;
    spat[0].lightState    = 3;
    spat[0].curCoutingTime = 70;
    spat[0].nextDurationTime = 20;

    MockV2xSpatInfo(spat, ADAS_SPAT_INFO_NUM);

    const CDDFusion::CddFusionRepo& cddData = DataRepo::GetInstance().GetCddFusionData();
    ASSERT_TRUE(cddData.spatInfo.trafficLightSt);
    ASSERT_EQ(cddData.spatInfo.redTime, 0);
    ASSERT_EQ(cddData.spatInfo.greenTime, spat[0].curCoutingTime);
    ASSERT_EQ(cddData.spatInfo.yellowTime, spat[0].curCoutingTime + spat[0].nextDurationTime);
}

TEST_F(V2xDataFusionTest, normal_spatinfo_with_current_yellow_light_test)
{
    V2X::AdasSpatInfo spat[ADAS_SPAT_INFO_NUM];
    spat[0].spatInfoValid = true;
    spat[0].lightState    = 7;
    spat[0].curCoutingTime = 70;
    spat[0].nextDurationTime = 20;
    MockV2xSpatInfo(spat, ADAS_SPAT_INFO_NUM);

    const CDDFusion::CddFusionRepo& cddData = DataRepo::GetInstance().GetCddFusionData();
    ASSERT_TRUE(cddData.spatInfo.trafficLightSt);
    ASSERT_EQ(cddData.spatInfo.yellowTime, 0);
    ASSERT_EQ(cddData.spatInfo.redTime, spat[0].curCoutingTime);
    ASSERT_EQ(cddData.spatInfo.greenTime, spat[0].curCoutingTime + spat[0].nextDurationTime);
}

TEST_F(V2xDataFusionTest, normal_spatinfo_with_current_unknown_light_test)
{
    V2X::AdasSpatInfo spat[ADAS_SPAT_INFO_NUM];
    spat[0].spatInfoValid = true;
    spat[0].lightState    = 1;
    spat[0].curCoutingTime = 70;
    spat[0].nextDurationTime = 20;
    MockV2xSpatInfo(spat, ADAS_SPAT_INFO_NUM);

    const CDDFusion::CddFusionRepo& cddData = DataRepo::GetInstance().GetCddFusionData();
    ASSERT_FALSE(cddData.spatInfo.trafficLightSt);
    ASSERT_EQ(cddData.spatInfo.yellowTime, 0xFFFF);
    ASSERT_EQ(cddData.spatInfo.redTime, 0xFFFF);
    ASSERT_EQ(cddData.spatInfo.greenTime, 0xFFFF);
}

TEST_F(V2xDataFusionTest, normal_addmapresult_test)
{
    V2X::MapAddResult extraMap;
    extraMap.offsetTolink = 10;
    extraMap.offsetTolane = 7;
    extraMap.distToNode = 200;
    extraMap.isAtAcross = false;

    MockV2xExtraMap(extraMap);
}

TEST_F(V2xDataFusionTest, normal_egomap_test)
{
    V2X::EgoVehMapInfo hostVehiMap;
    MockV2xHostVehiMap(hostVehiMap);
}

TEST_F(V2xDataFusionTest, normal_objectmap_test)
{
    V2X::ObjVehMapInfo objMap[ADAS_OBJ_VEH_INFO_NUM];
    uint32_t* ptr = reinterpret_cast<uint32_t*>(&objMap[0]);
    for (uint32_t i=0; i<sizeof(objMap)/sizeof(uint32_t); i++)
    {
        ptr[i] = rand();
    }

    MockV2xObjVehiMap(objMap, ADAS_OBJ_VEH_INFO_NUM);
}

TEST_F(V2xDataFusionTest, normal_gSentry_warning_test)
{
    V2X::WarningInfo gSentryWarning;
    gSentryWarning.level = 1;
    gSentryWarning.objectCollisionTTC = 60;
    gSentryWarning.remoteBsmId = 24;
    gSentryWarning.remoteLocalId = 204;
    gSentryWarning.warningType = 1;

    MockV2xgSentryWarn(gSentryWarning);

    const CDDFusion::CddFusionRepo& cddData = DataRepo::GetInstance().GetCddFusionData();
    ASSERT_EQ(cddData.gSentryWarningInfo.level, gSentryWarning.level);
    ASSERT_EQ(cddData.gSentryWarningInfo.warningType, gSentryWarning.warningType);
    ASSERT_EQ(cddData.gSentryWarningInfo.targetID, gSentryWarning.remoteLocalId);
}

TEST_F(V2xDataFusionTest, normal_object_vehicle_info_test)
{
    /* 本用例测试数据从prescan获取，因此校验值使用魔法数
                本车long 114.0016624, lat 29.9985554，X 160.41， Y -160.13, 朝正北方向行驶
                远车long 114.0017942, lat 29.9987438，X 173.11， Y -139.25, 朝正西方向行驶, 前向速度10m/s，无转弯
                远车在本车坐标系描述为：                 X 12.7，   Y 20.88, 速度-10m/S
    */ 
    V2X::AdasObjVehInfo objVehi[ADAS_OBJ_VEH_INFO_NUM];
    V2X::AdasObjVehInfo& testVehi = objVehi[0];
    testVehi.timeStamp = 14676655565;
    testVehi.localId   = 56;
    testVehi.objectSource = 2;
    testVehi.vehicleClass = 11;
    testVehi.size.length  = 300;
    testVehi.size.width   = 250;
    testVehi.size.width   = 150;
    testVehi.objectHeadingAngle = 21600;
    testVehi.objectYawAngle = 0;
    testVehi.gear = 2;
    testVehi.steeringWheelAngle = 0;
    testVehi.remoteLight = 0;
    testVehi.speed = 500;
    testVehi.accelSet.latitude = 0;
    testVehi.accelSet.longitude = 200;
    testVehi.vehicelPos.longitude = 1140017942;
    testVehi.vehicelPos.latitude  = 299987438;
    testVehi.vehicelPos.elevation = 40000;

    MockV2xObjVehiInfo(objVehi, ADAS_OBJ_VEH_INFO_NUM);
    const CDDFusion::CddFusionRepo& cddData = DataRepo::GetInstance().GetCddFusionData();
    
    ASSERT_NEAR(12.7,  cddData.v2xObjVehi[0].dx, 0.2);
    ASSERT_NEAR(20.88, cddData.v2xObjVehi[0].dy, 0.2);
    ASSERT_NEAR(0, cddData.v2xObjVehi[0].vx, 0.2);
    ASSERT_NEAR(-testVehi.speed * 0.02, cddData.v2xObjVehi[0].vy, 0.2);
    ASSERT_NEAR(0, cddData.v2xObjVehi[0].ax, 0.2);
    ASSERT_NEAR(-testVehi.accelSet.longitude * 0.01, cddData.v2xObjVehi[0].ay, 0.2);

}
TEST_F(V2xDataFusionTest, normal_objvehi_info_with_error_can_test)
{
    /* 本用例测试数据从prescan获取，因此校验值使用魔法数
                本车long 114.0016624, lat 29.9985554，X 160.41， Y -160.13, 朝正北方向行驶
                远车long 114.0017942, lat 29.9987438，X 173.11， Y -139.25, 朝正西方向行驶, 前向速度10m/s，无转弯
                远车在本车坐标系描述为：                 X 12.7，   Y 20.88, 速度-10m/S
    */ 

    CAN::HostVehiclePos host;
    MockHostVehiInfo(host);
    V2X::AdasObjVehInfo objVehi[ADAS_OBJ_VEH_INFO_NUM];
    MockV2xObjVehiInfo(objVehi, ADAS_OBJ_VEH_INFO_NUM);

}

TEST_F(V2xDataFusionTest, falut_branch_test)
{
    uint8_t buff[10 * 1024];
    V2X::V2xAdasMsgHeader* head = new(&buff[0]) V2X::V2xAdasMsgHeader();
    head->msgId = EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT;
    head->msgLen = 5;

    uint16_t errorLen = sizeof(V2X::V2xAdasMsgHeader) + head->msgLen;

    EventMessage msg_status = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), errorLen};
    EventDispatcher::ProcessMessage(msg_status);

    head->msgId = EV_GSENTRY_ADAS_CALC_MAPINFO_REPORT;
    EventMessage msg_extramap = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), errorLen};
    EventDispatcher::ProcessMessage(msg_extramap);

    head->msgId = EV_GSENTRY_ADAS_SPATINFO_REPORT;
    EventMessage msg_spat = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), errorLen};
    EventDispatcher::ProcessMessage(msg_spat);

    head->msgId = EV_GSENTRY_ADAS_OBJECT_VEHICLE_REPORT;
    EventMessage msg_objvehi = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), errorLen};
    EventDispatcher::ProcessMessage(msg_objvehi);

    head->msgId = EV_GSENTRY_ADAS_EGOVEHI_MAPINFO_REPORT;
    EventMessage msg_egovehi = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), errorLen};
    EventDispatcher::ProcessMessage(msg_egovehi);

    head->msgId = EV_GSENTRY_ADAS_OBJVEHI_MAPINFO_REPORT;
    EventMessage msg_objmap = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), errorLen};
    EventDispatcher::ProcessMessage(msg_objmap);

    head->msgId = EV_GSENTRY_ADAS_WARNING_REPORT;
    EventMessage msg_warn = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), errorLen};
    EventDispatcher::ProcessMessage(msg_warn);

    head->msgId = 4647;
    EventMessage msg_errormsg = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), errorLen};
    EventDispatcher::ProcessMessage(msg_errormsg);

    EventMessage msg_nullptr = {MsgType::V2X, nullptr, errorLen};
    EventDispatcher::ProcessMessage(msg_nullptr);
}


