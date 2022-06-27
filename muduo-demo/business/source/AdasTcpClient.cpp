#include "AdasTcpClient.h"
#include "V2xAdasEventMacro.h"
#include "V2xDataDispatcher.h"
#include "V2xDataStruct.h"

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
    msgDispatcher[EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT] = std::bind(&V2xDataDispatcher::ProcessGSentrySatatus, _1, _2);
    msgDispatcher[EV_GSENTRY_ADAS_CALC_MAPINFO_REPORT] = std::bind(&V2xDataDispatcher::ProcessCalcMapResult, _1, _2);
    msgDispatcher[EV_GSENTRY_ADAS_SPATINFO_REPORT] = std::bind(&V2xDataDispatcher::ProcessSpatInfo, _1, _2);
    msgDispatcher[EV_GSENTRY_ADAS_OBJECT_VEHICLE_REPORT] = std::bind(&V2xDataDispatcher::ProcessObjVehiInfo, _1, _2);
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
    // muduo send big endian data, actually develper promise send little endian data
    V2xAdasMsgHeader head;
    head.msgId = static_cast<uint16_t>(ntohs(buf->readInt16()));
    LOG_INFO << "recieve buff: msgid = " << head.msgId; 

    if (msgDispatcher.find(head.msgId) != msgDispatcher.end())
    {
        // demo版本，未取出header所有数据
        msgDispatcher[head.msgId](reinterpret_cast<char*>(&head), static_cast<u_int16_t>(sizeof(head)));
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
