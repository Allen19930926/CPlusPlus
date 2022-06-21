#include "AdasTcpServer.h"
#include <functional>
#include <algorithm>

using namespace muduo;
using namespace muduo::net;
using namespace std::placeholders;

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

void AdasTcpServer::periodCb()
{
    auto cb = [this](TcpConnectionPtr conn)
    {
        if (conn->connected())
        {
            conn->send("Timely Hello!");
        }
    };
    std::for_each(connections_.begin(), connections_.end(), cb);
}