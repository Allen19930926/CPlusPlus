#include "AdasTcpClient.h"

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
    msgDispatcher[EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT] = std::bind(&V2xDataDispatcher::ProcessGSentrySatatus, this, _1, _2);
    msgDispatcher[EV_GSENTRY_ADAS_CALC_MAPINFO_REPORT] = std::bind(&V2xDataDispatcher::ProcessCalcMapResult, this, _1, _2);
    msgDispatcher[EV_GSENTRY_ADAS_SPATINFO_REPORT] = std::bind(&V2xDataDispatcher::ProcessSpatInfo, this, _1, _2);
    msgDispatcher[EV_GSENTRY_ADAS_OBJECT_VEHICLE_REPORT] = std::bind(&V2xDataDispatcher::ProcessObjVehiInfo, this, _1, _2);
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
    buf->retrieveInt32();
    uint16_t msgId = static_cast<uint16_t>(buf->readInt16());
    if (msgDispatcher.find(msgId) != msgDispatcher.end())
    {
        // demo版本，未取出header所有数据
        msgDispatcher[msgId](buf->peek(), buf->readableBytes());
    }

}

void AdasTcpClient::connect()
{
    client_.connect();
}

void AdasTcpClient::disconnect()
{
    client_.disconnect();
}
