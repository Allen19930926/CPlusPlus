#include "gSentry_proxy.h"

using std::string;

GSentryProxy::GSentryProxy(asio::io_context& ioService, string ipAddr, string port)
        : IProxy(ioService, MsgType::V2X), client(ioService, MsgType::V2X, ipAddr, port)
{

}

void GSentryProxy::Init()
{
    client.start();
    KeepAlive(1000);
}

void GSentryProxy::KeepAlive(const uint32_t interval)
{
    SetPeriodTask(interval, std::bind(&GSentryProxy::WriteClientAliveMsg, this));
}

void GSentryProxy::WriteClientAliveMsg()
{
    std::string msg = "gSentry(client) keep alive!! gSentry(client) keep alive!! ";
    client.write(msg);
}

void GSentryProxy::DoClientWrite(const char* buf , const uint16_t len)
{
    DefaultChatMessage msg(buf, len);
    client.write(msg);
}

// for callback demo
void GSentryProxy::WritePeriodData(const uint32_t interval)
{
    SetPeriodTask(interval, std::bind(&GSentryProxy::WriteData, this));
}

void GSentryProxy::WriteData()
{
    std::string msg = "gSentry proxy period write";
    client.write(msg);
}
