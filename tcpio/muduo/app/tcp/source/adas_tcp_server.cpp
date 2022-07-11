#include "adas_tcp_server.h"
#include <functional>
#include <algorithm>
#include "data_base.h"
#include "event_queue.h"

//for test
#include "v2x_data_struct.h"

using namespace muduo;
using namespace muduo::net;
using namespace std::placeholders;
using namespace V2X;


AdasTcpServer::AdasTcpServer(EventLoop* loop, const InetAddress& listenAddr)
: server_(loop, listenAddr, "ChatServer", TcpServer::Option::kReusePort)
{
    server_.setConnectionCallback(
        std::bind(&AdasTcpServer::onConnection, this, _1));
  server_.setMessageCallback(
      std::bind(&AdasTcpServer::onMessage, this, _1, _2, _3));
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
    // client剥离V2xAdasMsgHeader头， 读取msgId.   muduo库需要更换，因此本处先略过
    CDD_FUSION_EVENT_QUEUE.push({0x101, buf->peek(), static_cast<uint16_t>(buf->readableBytes())});

}