#include "can_proxy.h"

CanProxy::CanProxy(asio::io_context& ioService, short port) : IProxy(ioService, MsgType::CAN), server(ioService, MsgType::CAN, port)
{

}

void CanProxy::Init()
{
    server.start();
    KeepAlive(1000);
}

void CanProxy::KeepAlive(const uint32_t interval)
{
    SetPeriodWriteTask(TCP_SERVER, interval, "CAN(server) keep alive!! CAN(server) keep alive!! ");
}

void CanProxy::DoPeriodServerWriteTask(PeriodTimer timer ,const uint32_t interval, std::string msg)
{
    server.write(msg);
    timer->expires_after(std::chrono::milliseconds(interval));
    timer->async_wait(std::bind(&CanProxy::DoPeriodServerWriteTask, this, timer, interval, msg));
}

void CanProxy::DoServerWrite(const char* buf , const uint16_t len)
{
    DefaultChatMessage msg(buf, len);
    server.write(msg);
}

