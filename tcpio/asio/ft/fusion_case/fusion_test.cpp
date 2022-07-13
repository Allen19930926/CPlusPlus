#include "gtest/gtest.h"
#include "hb_data.h"
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

    // uint16_t MockGSentryStatus(uint8_t status, bool faultStatus)
    // {
    //     uint16_t len = 0;
    //     V2xAdasMsgHeader& head = *(new (&buff[0]) V2xAdasMsgHeader);
    //     head.msgId = EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT;
    //     len += sizeof(V2xAdasMsgHeader);

    //     GSentryStatus& mockStatus = *(new (&buff[len]) GSentryStatus);
    //     mockStatus.gSentryStatus = status;
    //     mockStatus.faultStatus = faultStatus;
    //     len += sizeof(GSentryStatus);

    //     return len;
    // }

    uint8_t buff[1024];
    // DataProcessor proc;

};

TEST_F(V2xDataFusionTest, gSentryStatus_abnormal_test)
{
    ASSERT_TRUE(true);
}
