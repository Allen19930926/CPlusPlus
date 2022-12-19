#include "fusion_asio_udp_server.h"
#include <glog/logging.h>

FusionAsioUdpServer::FusionAsioUdpServer(asio::io_context& io_context, short port, UdpReadCB cb)
           : io_context_(io_context), socket_(io_context, udp::endpoint(asio::ip::address_v4::any(), port)),
             read_call_back(cb)
{
    do_receive();
}

void FusionAsioUdpServer::do_receive()
{
    socket_.async_receive_from(
        asio::buffer(data_, UDP_MAX_READ_LEN), remote_endpoint_,
            [this](std::error_code ec, std::size_t bytes)
        {
            if (!ec && bytes!= 0)
            {
                read_call_back(&data_[0], bytes);
            }
            do_receive();
    });
}
