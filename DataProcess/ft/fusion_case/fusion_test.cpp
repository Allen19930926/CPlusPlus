#include "gtest/gtest.h"
#include "hb_data.h"
#include "data_process.h"
#include "V2xAdasEventMacro.h"
#include "V2xDataStruct.h"
#include <iostream>
#include <new>

using std::cout;
using std::endl;

struct V2xDataFusionTest : testing::Test
{
    void SetUp()
    {
        cout << "case up!" << endl;
    }

    void TearDown()
    {
        memset(buff,0,sizeof(buff));
        cout << "case down!" << endl;
    }

    uint16_t MockGSentryStatus(uint8_t status, bool faultStatus)
    {
        uint16_t len = 0;
        V2xAdasMsgHeader& head = *(new (&buff[0]) V2xAdasMsgHeader);
        head.msgId = EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT;
        len += sizeof(V2xAdasMsgHeader);

        GSentryStatus& mockStatus = *(new (&buff[len]) GSentryStatus);
        mockStatus.gSentryStatus = status;
        mockStatus.faultStatus = false;
        len += sizeof(GSentryStatus);

        return len;
    }

    uint8_t buff[1000];
    DataProcessor proc;

};

TEST_F(V2xDataFusionTest, gSentryStatus_abnormal_test)
{
    ASSERT_TRUE(proc.IsGsentryWork());
    uint16_t len = MockGSentryStatus(4, false);
    proc.ProcessV2xData(&buff[0], len);
    ASSERT_FALSE(proc.IsGsentryWork());
}
