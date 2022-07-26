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

AdasAsioTcpSession::AdasAsioTcpSession(tcp::socket socket, asio::io_context& io_context, MsgType type)
    : socket_(std::move(socket)), msgType(type)
{
    socket_.set_option(asio::ip::tcp::socket::reuse_address(true));
}

void AdasAsioTcpSession::start()
{
    do_read();
}

void AdasAsioTcpSession::do_write(std::string msg)
{
    socket_.async_write_some(asio::buffer(msg.data(), msg.length()),
        [this, msg](std::error_code ec, std::size_t length)
        {
            if (!ec) 
            {
                std::cout << "server session write success! msg: " << msg << std::endl;
            }
        });
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


AdasAsioTcpServer::AdasAsioTcpServer(asio::io_context& io_context, MsgType type, short port)
: io_context_(io_context), acceptor_(io_context, tcp::endpoint(tcp::v4(), port)), msgType(type)
{
}

void AdasAsioTcpServer::start()
{
    acceptor_.async_accept(
        [this](std::error_code ec, tcp::socket socket)
        {
            if (!ec)
            {
                std::shared_ptr<AdasAsioTcpSession> activeSession = 
                                std::make_shared<AdasAsioTcpSession>(std::move(socket), io_context_, msgType);
                session = activeSession;
                activeSession->start();
            }

            start();
        });
}

void AdasAsioTcpServer::write(std::string msg)
{
    // 目前设计server和client是一对一，因此server设计只与最后一次成功连接的client进行写入会话
    if (session.expired())
    {
        return ;
    }
    std::shared_ptr<AdasAsioTcpSession> activeSession = session.lock();
    activeSession->do_write(msg);
}

