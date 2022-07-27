#include "gSentry_proxy.h"

using std::string;

GSentryProxy::GSentryProxy(asio::io_context& ioService, string ipAddr, string port)
        : IProxy(ioService, MsgType::V2X), client(ioService, MsgType::V2X, ipAddr, port)
{

}

void GSentryProxy::Init()
{
    client.start();
    // KeepAlive(1000);
}

void GSentryProxy::KeepAlive(const uint32_t interval)
{
    SetPeriodWriteTask(TCP_CLIENT, interval, "gSentry(client) keep alive!! gSentry(client) keep alive!! ");
}

void GSentryProxy::DoPeriodClientWriteTask(PeriodTimer timer ,const uint32_t interval, string msg)
{
    client.write(msg);
    timer->expires_after(std::chrono::milliseconds(interval));
    timer->async_wait(std::bind(&GSentryProxy::DoPeriodClientWriteTask, this, timer, interval, msg));
}

void GSentryProxy::DoClientWrite(const char* buf , const uint16_t len)
{
    DefaultChatMessage msg(buf, len);
    client.write(msg);
}
