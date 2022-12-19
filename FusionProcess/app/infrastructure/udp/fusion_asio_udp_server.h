#ifndef E7FCD159_2617_40BB_9DD9_0417EB62EFE8
#define E7FCD159_2617_40BB_9DD9_0417EB62EFE8

#include "asio.hpp"
#include <string>
#include <functional>

using asio::ip::udp;
using UdpReadCB =  std::function<void(char*, uint32_t)>;

class FusionAsioUdpServer
{
public:
    static const uint32_t UDP_MAX_READ_LEN = 10 * 1024;
public:
    FusionAsioUdpServer(asio::io_context& io_context, short port, UdpReadCB cb);
private:
    void do_receive();

private:
    asio::io_context& io_context_;
    udp::socket socket_;
    udp::endpoint remote_endpoint_;
    char data_[UDP_MAX_READ_LEN];
    UdpReadCB read_call_back;
};

#endif /* E7FCD159_2617_40BB_9DD9_0417EB62EFE8 */
