#ifndef D2A0665A_3989_4E30_9E59_189463A188F9
#define D2A0665A_3989_4E30_9E59_189463A188F9

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
    void SetUp();
    void TearDown();
    void MockHostVehiInfo(const CAN::HostVehiclePos& host);
    void MockGSentryStatus(V2X::GSentryStatus& status);
    uint16_t MockV2xHead(uint16_t msgId, uint32_t msgLen);
    void MockV2xSpatInfo(V2X::AdasSpatInfo spat[], uint16_t num);
    void MockV2xExtraMap(const V2X::MapAddResult& extarMap);
    void MockV2xHostVehiMap(const V2X::EgoVehMapInfo& hostMap);
    void MockV2xObjVehiMap(V2X::ObjVehMapInfo objMap[], uint16_t num);
    void MockV2xgSentryWarn(const V2X::WarningInfo& gSentryWarn);
    void MockV2xObjVehiInfo(V2X::AdasObjVehInfo objVehi[], uint16_t num);
private:
    uint8_t buff[1024 * 10];

};

#endif /* D2A0665A_3989_4E30_9E59_189463A188F9 */
