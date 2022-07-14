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
        MockV2xHead(EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT, sizeof(status));

        memcpy(&buff[sizeof(V2X::V2xAdasMsgHeader)], &status, sizeof(status));
        EventMessage msg = {MsgType::V2X, reinterpret_cast<const char*>(&buff[0]), sizeof(V2X::V2xAdasMsgHeader) + sizeof(status)};
        EventDispatcher::ProcessMessage(msg);
        const V2X::V2xData& v2xData = DataRepo::GetInstance().GetV2xData();
        ASSERT_EQ(status.faultStatus, v2xData.status.faultStatus);
        ASSERT_EQ(status.gSentryStatus, v2xData.status.gSentryStatus);
    }

    void MockV2xHead(uint16_t msgId, uint32_t msgLen)
    {
        V2X::V2xAdasMsgHeader* head = new(&buff[0]) V2X::V2xAdasMsgHeader();
        head->msgId = msgId;
        head->msgLen = msgLen;
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

TEST_F(V2xDataFusionTest, with_fault_gSentryStatus_test)
{
    V2X::GSentryStatus status = {4,true};
    MockGSentryStatus(status);
}
