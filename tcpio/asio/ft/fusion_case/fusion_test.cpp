#include "gtest/gtest.h"
#include "data_base.h"
#include "event_msg.h"
#include "event_dispatcher.h"
#include <new>
#include "v2x_adas_event_macro.h"

using std::cout;
using std::endl;

struct V2xDataFusionTest : testing::Test
{
    void SetUp()
    {
        CAN::HostVehiclePos host = {299985554,1140016624,40000,0};
        MockHostVehiInfo(host);
        V2X::GSentryStatus status = {3,false};
        MockGSentryStatus(status);
    }

    void TearDown()
    {
        DataRepo::GetInstance().Clear();
        memset(buff,0,sizeof(buff));
    }

    void MockHostVehiInfo(const CAN::HostVehiclePos& host)
    {
        EventMessage msg = {MsgType::CAN, reinterpret_cast<const char*>(&host), sizeof(host)};
        EventDispatcher::ProcessMessage(msg);
        const CAN::HostVehiclePos& cmpHost = DataRepo::GetInstance().GetHostVehicle();
        ASSERT_EQ(host.elevation, cmpHost.elevation);
        ASSERT_EQ(host.latitude, cmpHost.latitude);
        ASSERT_EQ(host.longitude, cmpHost.longitude);
        ASSERT_EQ(host.objectHeadingAngle, cmpHost.objectHeadingAngle);
    }

    void MockGSentryStatus(V2X::GSentryStatus& status)
    {
        uint16_t len = MockV2xHead(EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT, sizeof(status));

        memcpy(&buff[len], &status, sizeof(status));
        EventMessage msg = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), static_cast<uint16_t>(len + sizeof(status))};
        EventDispatcher::ProcessMessage(msg);
        const V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
        ASSERT_EQ(status.faultStatus, v2xData.status.faultStatus);
        ASSERT_EQ(status.gSentryStatus, v2xData.status.gSentryStatus);
    }

    uint16_t MockV2xHead(uint16_t msgId, uint32_t msgLen)
    {
        V2X::V2xAdasMsgHeader* head = new(&buff[0]) V2X::V2xAdasMsgHeader();
        head->msgId = msgId;
        head->msgLen = msgLen;
        return sizeof(V2X::V2xAdasMsgHeader);
    }

    void MockV2xSpatInfo(V2X::AdasSpatInfo spat[], uint16_t num)
    {
        uint16_t payloadLen = sizeof(V2X::AdasSpatInfo) * num;
        uint16_t len = MockV2xHead(EV_GSENTRY_ADAS_SPATINFO_REPORT, payloadLen);
        memcpy(&buff[len], &spat[0], payloadLen);

        EventMessage msg = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), static_cast<uint16_t>(len + payloadLen)};
        EventDispatcher::ProcessMessage(msg);
    }

    uint8_t buff[1024];

};

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
    // MockV2xSpatInfo(spat, ADAS_SPAT_INFO_NUM);

    // const CDDFusion::CddFusionRepo& cddData = DataRepo::GetInstance().GetCddFusionData();
    // ASSERT_FALSE(cddData.);
    // ASSERT_EQ(cddData.spatInfo.yellowTime, 0xFFFF);
    // ASSERT_EQ(cddData.spatInfo.redTime, 0xFFFF);
    // ASSERT_EQ(cddData.spatInfo.greenTime, 0xFFFF);
}

