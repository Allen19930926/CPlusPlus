#include "hmi_proxy.h"
#include "hmi_data.h"

HmiProxy::HmiProxy(asio::io_context& ioService, std::string clientIp, std::string clientPort, short listenPort)
            : IProxy(ioService, MsgType::HMI), server(ioService, MsgType::HMI, listenPort),
              client(ioService, MsgType::HMI, clientIp, clientPort)
{

}

void HmiProxy::Init()
{
    server.start();
    client.start();
    KeepAlive(1000);
}


void HmiProxy::KeepAlive(const uint32_t interval)
{
    SetPeriodTask(interval, std::bind(&HmiProxy::WriteServerAliveMsg, this));
    SetPeriodTask(interval, std::bind(&HmiProxy::WriteClientAliveMsg, this));
}

void HmiProxy::WriteServerAliveMsg()
{
    
}

void HmiProxy::WriteClientAliveMsg()
{
    gSentryKeepAliveRequestFrame frame;
    frame.tag = HMI_TAG_KEEP_ALIVE_REQ;
    json j;
    to_json(j, frame);
    client.write(j.dump());
}

void HmiProxy::DoClientWrite(const char* buf , const uint16_t len)
{
    DefaultChatMessage msg(buf, len);
    client.write(msg);
}

void HmiProxy::DoServerWrite(const char* buf , const uint16_t len)
{
    DefaultChatMessage msg(buf, len);
    server.write(msg);
}
