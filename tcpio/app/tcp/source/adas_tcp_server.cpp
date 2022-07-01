#include "adas_tcp_server.h"
#include <functional>
#include <algorithm>
#include "data_base.h"
#include <condition_variable>
#include <queue>
#include <mutex>

//for test
#include "v2x_data_struct.h"

using namespace muduo;
using namespace muduo::net;
using namespace std::placeholders;
using namespace V2X;
extern std::mutex mtx;
extern std::condition_variable cv;
extern std::queue<CDDFusion::EventMessage> eventQueue;


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
    std::unique_lock<std::mutex> lck(mtx);
    // client剥离V2xAdasMsgHeader头， 读取msgId.   muduo库需要更换，因此本处先略过
    eventQueue.emplace(0x101, buf->peek(), buf->readableBytes());
    printf("queue size is %lu\n", eventQueue.size());
    cv.notify_all();

}