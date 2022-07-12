#include "hmi_proxy.h"
#include "hmi_data.h"

HmiProxy::HmiProxy(asio::io_context& ioService, std::string clientIp, std::string clientPort, short listenPort)
            : server(ioService, MsgType::HMI, listenPort), client(ioService, MsgType::HMI, clientIp, clientPort)
{
    ConstructServerAliveMsg();
    ConstructClientAliveMsg();
}

void HmiProxy::Start()
{
    server.start();
    client.start();
}

void HmiProxy::ConstructServerAliveMsg()
{

}

void HmiProxy::ConstructClientAliveMsg()
{
    gSentryKeepAliveRequestFrame frame;
    frame.tag = HMI_TAG_KEEP_ALIVE_REQ;
    json j;
    to_json(j, frame);
    clientAliveMsg = j.dump();
}

void HmiProxy::SetPeriodWriteTask(const uint32_t interval)
{
    server.SetPeriodWriteTask(interval, serverAliveMsg);
    client.SetPeriodWriteTask(interval, clientAliveMsg);
}
