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
    SetPeriodTask(interval, std::bind(&CanProxy::WriteServerAliveMsg, this));
}

void CanProxy::WriteServerAliveMsg()
{
    std::string msg = "CAN(server) keep alive!! CAN(server) keep alive!! ";
    // server.write(msg);
}

void CanProxy::DoServerWrite(const char* buf , const uint16_t len)
{
    DefaultChatMessage msg(buf, len);
    server.write(msg);
}

