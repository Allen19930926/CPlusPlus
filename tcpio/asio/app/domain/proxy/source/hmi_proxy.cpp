#include "hmi_proxy.h"
#include "hmi_data.h"

HmiProxy::HmiProxy(asio::io_context& ioService, std::string clientIp, std::string clientPort, short listenPort)
            : IProxy(ioService), server(ioService, MsgType::HMI, listenPort),
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
    const std::string clientAliveMsg = ConstructServerAliveMsg();
    const std::string serverAliveMsg = ConstructClientAliveMsg();
    SetPeriodWriteTask(TCP_CLIENT, interval, clientAliveMsg);
    SetPeriodWriteTask(TCP_SERVER, interval, serverAliveMsg);
}

std::string HmiProxy::ConstructServerAliveMsg()
{
    return "";
}

std::string HmiProxy::ConstructClientAliveMsg()
{
    gSentryKeepAliveRequestFrame frame;
    frame.tag = HMI_TAG_KEEP_ALIVE_REQ;
    json j;
    to_json(j, frame);
    return j.dump();
}

void HmiProxy::DoPeriodClientWriteTask(PeriodTimer timer ,const uint32_t interval, std::string msg)
{
    client.write(msg);
    timer->expires_after(std::chrono::milliseconds(interval));
    timer->async_wait(std::bind(&HmiProxy::DoPeriodClientWriteTask, this, timer, interval, msg));
}

void HmiProxy::DoPeriodServerWriteTask(PeriodTimer timer ,const uint32_t interval, std::string msg)
{
    server.write(msg);
    timer->expires_after(std::chrono::milliseconds(interval));
    timer->async_wait(std::bind(&HmiProxy::DoPeriodServerWriteTask, this, timer, interval, msg));
}
