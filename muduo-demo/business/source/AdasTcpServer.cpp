#include "AdasTcpServer.h"
#include <functional>
#include <algorithm>

//for test
#include "V2xDataStruct.h"

using namespace muduo;
using namespace muduo::net;
using namespace std::placeholders;

int msgid = 0x100;

AdasTcpServer::AdasTcpServer(EventLoop* loop, const InetAddress& listenAddr)
: server_(loop, listenAddr, "ChatServer")
{
    server_.setConnectionCallback(
        std::bind(&AdasTcpServer::onConnection, this, _1));
  server_.setMessageCallback(
      std::bind(&AdasTcpServer::onMessage, this, _1, _2, _3));
      loop->runEvery(1.0, std::bind(&AdasTcpServer::periodCb, this));
}

void AdasTcpServer::start()
{
    server_.start();
}

void AdasTcpServer::onConnection(const TcpConnectionPtr& conn)
{
    LOG_INFO << conn->peerAddress().toIpPort() << " -> "
                << conn->localAddress().toIpPort() << " is "
                << (conn->connected() ? "UP" : "DOWN");

    if (conn->connected())
    {
        connections_.insert(conn);
        return ;
    }
    connections_.erase(conn);
}

/*
    S32G act as a TCP server. Once CAN message comes, it will transfer the message to another module.
    Like echo.
*/
void AdasTcpServer::onMessage(const muduo::net::TcpConnectionPtr& conn,
                           muduo::net::Buffer* buf,
                           muduo::Timestamp time)
{
    muduo::string msg(buf->retrieveAllAsString());
    LOG_INFO << conn->name() << " recieve " << msg.size() << " bytes, "
            << "data received at " << time.toString();
    LOG_INFO << msg;

}

// test code,  mocking gSentry send data to S32G,  0x100 ~ 0x103

void AdasTcpServer::periodCb()
{
    V2xAdasMsgHeader head;
    head.msgId = msgid;
    auto cb = [this, &head](TcpConnectionPtr conn)
    {
        if (conn->connected())
        {
            conn->send(&head.msgId, sizeof(uint16_t));
            LOG_INFO << "send buff: msgid = " << head.msgId;
        }
    };
    std::for_each(connections_.begin(), connections_.end(), cb);
    if (msgid == 0x103)
    {
        msgid = 0x100;
    }
    else
    {
        msgid++;
    }
}