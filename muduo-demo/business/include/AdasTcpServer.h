#ifndef E869A85E_1F1F_463E_AFD3_C3D6360E60F8
#define E869A85E_1F1F_463E_AFD3_C3D6360E60F8

#include "../../ThirdParty/muduo/base/Logging.h"
#include "muduo/net/EventLoop.h"
#include "muduo/net/TcpServer.h"
#include <set>

using muduo::net::EventLoop;
using muduo::net::InetAddress;
using muduo::net::TcpConnectionPtr;
using muduo::net::TcpServer;

class AdasTcpServer
{
public:
    AdasTcpServer(EventLoop* loop, const InetAddress& listenAddr);
    ~AdasTcpServer() = default;
    AdasTcpServer(const AdasTcpServer& ref) = delete;
    AdasTcpServer& operator=(const AdasTcpServer& ref) = delete;
    void start();
private:
    void onConnection(const TcpConnectionPtr& conn);
    void onMessage(const muduo::net::TcpConnectionPtr& conn,
                           muduo::net::Buffer* buf,
                           muduo::Timestamp time);
private:
    typedef std::set<TcpConnectionPtr> ConnectionList;
    TcpServer server_;
    ConnectionList connections_;
};

#endif /* E869A85E_1F1F_463E_AFD3_C3D6360E60F8 */
