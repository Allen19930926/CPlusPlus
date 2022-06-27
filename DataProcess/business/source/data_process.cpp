#include "data_process.h"
#include "V2xAdasEventMacro.h"
#include "V2xDataStruct.h"
#include <iostream>

using std::cout;
using std::endl;
using namespace std::placeholders;

#define MAX_OBSERVE_OBJECT_NUM   20

DataProcessor::DataProcessor():gSentryStatus(3)
{
    dataDispatcher.insert(std::make_pair(EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT, std::bind(&DataProcessor::ProcessGSentryStatus, this, _1, _2)));
    gSentryObjs.reserve(MAX_OBSERVE_OBJECT_NUM);
    cameraObjs.reserve(MAX_OBSERVE_OBJECT_NUM);
}

void DataProcessor::ProcessV2xData(uint8_t* buf, uint16_t len)
{
    if (!buf || len<=sizeof(V2xAdasMsgHeader))
    {
        cout << "parameter check failed!!" << endl;
        return ;
    }

    V2xAdasMsgHeader& head = *reinterpret_cast<V2xAdasMsgHeader*>(buf);

    auto iter = dataDispatcher.find(head.msgId);
    if (iter != dataDispatcher.end())
    {
        iter->second(buf+sizeof(V2xAdasMsgHeader), len-sizeof(V2xAdasMsgHeader));
    }
}

void DataProcessor::ProcessGSentryStatus(uint8_t* buf, uint16_t len)
{
    if (!buf || len<sizeof(GSentryStatus))
    {
        cout << "parameter check failed!!" << endl;
        return ;
    }
    GSentryStatus& status = *reinterpret_cast<GSentryStatus*>(buf);
    gSentryStatus = status.gSentryStatus;
}
