#include "AdasTcpClient.h"
#include <functional>

using namespace muduo;
using namespace muduo::net;
using namespace std::placeholders;

AdasTcpClient::AdasTcpClient(EventLoop* loop, const InetAddress& serverAddr)
    : client_(loop, serverAddr, "ChatClient")
  {
    client_.setConnectionCallback(
        std::bind(&AdasTcpClient::onConnection, this, _1));
    client_.setMessageCallback(
        std::bind(&AdasTcpClient::onMessage, this, _1, _2, _3));
    client_.enableRetry();
  }

void AdasTcpClient::onConnection(const TcpConnectionPtr& conn)
{
    LOG_INFO << conn->localAddress().toIpPort() << " -> "
                << conn->peerAddress().toIpPort() << " is "
                << (conn->connected() ? "UP" : "DOWN");

    MutexLockGuard lock(mutex_);
    if (conn->connected())
    {
        connection_ = conn;
    }
    else
    {
        connection_.reset();
    }
}

void AdasTcpClient::onMessage(const TcpConnectionPtr& conn, Buffer* buf, Timestamp time)
{
    muduo::string msg(buf->retrieveAllAsString());
    LOG_INFO << "gSentry periodical message comes: " << msg;

}

void AdasTcpClient::connect()
{
    client_.connect();
}

void AdasTcpClient::disconnect()
{
    client_.disconnect();
}
