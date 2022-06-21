#ifndef FF8E8C98_64D8_447F_9304_DA4AF94B8C92
#define FF8E8C98_64D8_447F_9304_DA4AF94B8C92
#include "muduo/base/Logging.h"
#include "muduo/base/Mutex.h"
#include "muduo/net/EventLoop.h"
#include "muduo/net/TcpClient.h"
#include <map>
#include <functional>

using std::map;
using std::function;

class AdasTcpClient
{
public:
    AdasTcpClient(muduo::net::EventLoop* loop, const muduo::net::InetAddress& serverAddr);
    ~AdasTcpClient() = default;
    AdasTcpClient(const AdasTcpClient& ref) = delete;
    AdasTcpClient& operator=(const AdasTcpClient& ref) = delete;
    void connect();
    void disconnect();

private:
    void onConnection(const muduo::net::TcpConnectionPtr& conn);
    void onMessage(const muduo::net::TcpConnectionPtr& conn,
                           muduo::net::Buffer* buf,
                           muduo::Timestamp time);

private:
    muduo::net::TcpClient client_;
    muduo::MutexLock mutex_;
    muduo::net::TcpConnectionPtr connection_ GUARDED_BY(mutex_);
    std::map<int, function<void(const char*,uint16_t)>> msgDispatcher;
};

#endif /* FF8E8C98_64D8_447F_9304_DA4AF94B8C92 */
