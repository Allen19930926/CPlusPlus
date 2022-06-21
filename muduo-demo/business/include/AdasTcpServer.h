#ifndef E869A85E_1F1F_463E_AFD3_C3D6360E60F8
#define E869A85E_1F1F_463E_AFD3_C3D6360E60F8

#include "muduo/base/Logging.h"
#include "muduo/net/EventLoop.h"
#include "muduo/net/TcpServer.h"
#include <set>

class AdasTcpServer
{
public:
    AdasTcpServer(muduo::net::EventLoop* loop, const muduo::net::InetAddress& listenAddr);
    ~AdasTcpServer() = default;
    AdasTcpServer(const AdasTcpServer& ref) = delete;
    AdasTcpServer& operator=(const AdasTcpServer& ref) = delete;
    void start();
private:
    void onConnection(const muduo::net::TcpConnectionPtr& conn);
    void onMessage(const muduo::net::TcpConnectionPtr& conn,
                           muduo::net::Buffer* buf,
                           muduo::Timestamp time);
    void periodCb();
private:
    typedef std::set<muduo::net::TcpConnectionPtr> ConnectionList;
    muduo::net::TcpServer server_;
    ConnectionList connections_;
};

#endif /* E869A85E_1F1F_463E_AFD3_C3D6360E60F8 */
