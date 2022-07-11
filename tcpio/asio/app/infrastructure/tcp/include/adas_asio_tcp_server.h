#ifndef CD72DBDC_366D_4F08_B091_0522E40C3FE7
#define CD72DBDC_366D_4F08_B091_0522E40C3FE7

#include "asio.hpp"
#include "asio/steady_timer.hpp"

using asio::ip::tcp;

class AdasAsioTcpSession : public std::enable_shared_from_this<AdasAsioTcpSession>
{
public:
    AdasAsioTcpSession(tcp::socket socket);
    void start();

private:
    void do_read();

private:
    tcp::socket socket_;
    enum { max_length = 1024 };
    char data_[max_length];
};


class AdasAsioTcpServer
{
public:
  AdasAsioTcpServer(asio::io_context& io_context, short port);

private:
  void do_accept();

private:
  tcp::acceptor acceptor_;
};

#endif /* CD72DBDC_366D_4F08_B091_0522E40C3FE7 */
