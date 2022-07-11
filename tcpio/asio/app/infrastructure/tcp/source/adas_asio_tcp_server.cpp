#include "adas_asio_tcp_server.h"
#include <functional>
#include <iostream>
#include "v2x_adas_event_macro.h"
#include "event_dispatcher.h"
#include "v2x_data_struct.h"
#include "cdd_fusion.h"
#include "event_queue.h"

using namespace std::placeholders;
using namespace V2X;

AdasAsioTcpSession::AdasAsioTcpSession(tcp::socket socket) : socket_(std::move(socket))
{
    socket_.set_option(asio::ip::tcp::socket::reuse_address(true));
}

void AdasAsioTcpSession::start()
{
    do_read();
}

void AdasAsioTcpSession::do_read()
{
    auto self(shared_from_this());
    socket_.async_read_some(asio::buffer(data_, max_length),
        [this, self](std::error_code ec, std::size_t length)
        {
            uint16_t headlen = sizeof(V2xAdasMsgHeader);
            if (!ec && length >= headlen)
            {
                std::cout << "recieve msg : " << &data_[0] << std::endl;
                // client剥离V2xAdasMsgHeader头， 读取msgId.   muduo库需要更换，因此本处先略过
                CDD_FUSION_EVENT_QUEUE.push({0x101, 
                data_ + headlen, static_cast<uint16_t>(length - headlen)});
            }
        });
}


AdasAsioTcpServer::AdasAsioTcpServer(asio::io_context& io_context, short port)
: acceptor_(io_context, tcp::endpoint(tcp::v4(), port))
{
    do_accept();
}

void AdasAsioTcpServer::do_accept()
{
    acceptor_.async_accept(
        [this](std::error_code ec, tcp::socket socket)
        {
            if (!ec)
            {
                std::make_shared<AdasAsioTcpSession>(std::move(socket))->start();
            }

            do_accept();
        });
}
