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

AdasAsioTcpSession::AdasAsioTcpSession(tcp::socket socket, asio::io_context& io_context, MsgType type, uint32_t interval, std::string msg)
    : socket_(std::move(socket)), msgType(type), deadline_(io_context), aliveInterval(interval), aliveMsg(msg)
{
    socket_.set_option(asio::ip::tcp::socket::reuse_address(true));
}

void AdasAsioTcpSession::start()
{
    do_read();
    if (aliveInterval <= 10800)
    {
        do_period_write(aliveInterval, aliveMsg);
    }
}

void AdasAsioTcpSession::do_read()
{
    auto self(shared_from_this());
    socket_.async_read_some(asio::buffer(data_, max_length),
        [this, self](std::error_code ec, std::size_t length)
        {
            if (!ec)
            {
                CDD_FUSION_EVENT_QUEUE.push({msgType, data_ , static_cast<uint16_t>(length)});
            }
        });
}

void AdasAsioTcpSession::do_period_write(const uint32_t interval, std::string msg)
{
    auto self(shared_from_this());
    socket_.async_write_some(asio::buffer(msg.data(), msg.length() + 1),
        [this, self](std::error_code ec, std::size_t length)
        {
            if (!ec) 
            {
                std::cout << "server period write(line:49) " << aliveMsg << std::endl;
                deadline_.expires_after(std::chrono::milliseconds(aliveInterval));
                deadline_.async_wait(std::bind(&AdasAsioTcpSession::do_period_write, self, aliveInterval, aliveMsg));
            }
        });
}


AdasAsioTcpServer::AdasAsioTcpServer(asio::io_context& io_context, MsgType type, short port)
: io_context_(io_context), acceptor_(io_context, tcp::endpoint(tcp::v4(), port)), msgType(type), aliveInterval(0xFFFFFFFF)
{
}

void AdasAsioTcpServer::SetPeriodWriteTask(const uint32_t interval, std::string msg)
{
    aliveInterval = interval;
    aliveMsg = msg;
}

void AdasAsioTcpServer::start()
{
    acceptor_.async_accept(
        [this](std::error_code ec, tcp::socket socket)
        {
            if (!ec)
            {
                std::make_shared<AdasAsioTcpSession>(std::move(socket), io_context_, msgType, aliveInterval, aliveMsg)->start();
            }

            start();
        });
}
