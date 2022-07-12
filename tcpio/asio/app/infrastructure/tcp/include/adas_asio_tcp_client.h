#ifndef B24BA3E3_14EC_47C8_95B7_EC70F30AAA60
#define B24BA3E3_14EC_47C8_95B7_EC70F30AAA60

#include "asio.hpp"
#include "asio/steady_timer.hpp"
#include <string>
#include <functional>
#include "event_msg.h"

using asio::ip::tcp;

class AdasAsioTcpClient
{
public:
    AdasAsioTcpClient(asio::io_context& io_context, MsgType type, std::string ipAddr, std::string port);
    void close();
    void start();
    void SetPeriodWriteTask(const uint32_t interval, std::string msg);

private:
    void do_connect(const tcp::resolver::results_type& endpoints);
    void do_read();
    void do_write(std::string msg);
    void do_period_write(const uint32_t interval, std::string msg);
    void on_read(const std::error_code & ec, size_t bytes);

private:
    asio::io_context& io_context_;
    tcp::socket socket_;
    enum { max_length = 1024 };
    char data_[max_length];
    asio::steady_timer deadline_;
    tcp::resolver::results_type endpoints_;
    MsgType msgType;
    uint32_t aliveInterval;
    std::string aliveMsg;
};


#endif /* B24BA3E3_14EC_47C8_95B7_EC70F30AAA60 */
