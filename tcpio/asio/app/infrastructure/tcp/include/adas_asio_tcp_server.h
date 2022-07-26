#ifndef CD72DBDC_366D_4F08_B091_0522E40C3FE7
#define CD72DBDC_366D_4F08_B091_0522E40C3FE7

#include "asio.hpp"
#include "asio/steady_timer.hpp"
#include "event_msg.h"
#include <string>

using asio::ip::tcp;

class AdasAsioTcpSession : public std::enable_shared_from_this<AdasAsioTcpSession>
{
public:
    AdasAsioTcpSession(tcp::socket socket, asio::io_context& io_context, MsgType type);
    void start();
    void do_write(std::string msg);

private:
    void do_read();

private:
    tcp::socket socket_;
    enum { max_length = 1024 };
    char data_[max_length];
    MsgType msgType;
};


class AdasAsioTcpServer
{
public:
    AdasAsioTcpServer(asio::io_context& io_context, MsgType type, short port);
    void start();
    void write(std::string msg);

private:

private:
    asio::io_context& io_context_;
    tcp::acceptor acceptor_;
    MsgType msgType;
    std::weak_ptr<AdasAsioTcpSession> session;
};

#endif /* CD72DBDC_366D_4F08_B091_0522E40C3FE7 */
