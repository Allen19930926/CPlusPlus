#include "fusion_asio_tcp_server.h"


FusionAsioTcpSession::FusionAsioTcpSession(asio::io_context& io, tcp::socket socket, asio::io_context& io_context, ReadCB cb)
: io_context_(io), socket_(std::move(socket)), read_call_back(cb)
{
    socket_.set_option(asio::ip::tcp::socket::reuse_address(true));
}
void FusionAsioTcpSession::start()
{
    doReadHeader();
}

void FusionAsioTcpSession::write(const DefaultChatMessage& msg)
{
    auto self(this->shared_from_this());
    asio::post(io_context_, [this, self, msg]()
    {
        bool write_in_progress = !writeMsgs.empty();
        writeMsgs.push_back(msg);
        if (!write_in_progress)
        {
            do_write();
        }
    });
}

void FusionAsioTcpSession::doReadHeader()
{
    auto self(this->shared_from_this());
    asio::async_read(socket_, asio::buffer(readMsg.Data(), FusionChatMessage::HeaderLength),
        [this, self](std::error_code ec, std::size_t /*length*/)
        {
            if (!ec && readMsg.DecodeHeader())
            {
                doReadBody();
            }
    });
}

void FusionAsioTcpSession::doReadBody()
{
    auto self(this->shared_from_this());
    asio::async_read(socket_, asio::buffer(readMsg.Body(), readMsg.BodyLength()),
        [this, self](std::error_code ec, std::size_t /*length*/)
        {
        if (!ec)
        {
            read_call_back(std::move(readMsg));
            doReadHeader();
        }
    });
}

void FusionAsioTcpSession::do_write()
{
    auto self(this->shared_from_this());
    asio::async_write(socket_, asio::buffer(writeMsgs.front().Data(), writeMsgs.front().Length()),
        [this, self](std::error_code ec, std::size_t /*length*/)
        {
            if (!ec)
            {
                writeMsgs.pop_front();
                if (!writeMsgs.empty())
                {
                    do_write();
                }
            }
    });
}


FusionAsioTcpServer::FusionAsioTcpServer(asio::io_context& io_context, short port, ReadCB cb)
: io_context_(io_context), acceptor_(io_context, tcp::endpoint(tcp::v4(), port)), read_call_back(cb)
{
}

void FusionAsioTcpServer::start()
{
    acceptor_.async_accept(
        [this](std::error_code ec, tcp::socket socket)
        {
            if (!ec)
            {
                std::shared_ptr<FusionAsioTcpSession> activeSession = 
                        std::make_shared<FusionAsioTcpSession>(io_context_, std::move(socket), io_context_, read_call_back);
                session = activeSession;
                activeSession->start();
            }

            start();
        });
}

void FusionAsioTcpServer::write(DefaultChatMessage msg)
{
    // 目前设计server和client是一对一，因此server设计只与最后一次成功连接的client进行写入会话
    if (session.expired())
    {
        return ;
    }
    std::shared_ptr<FusionAsioTcpSession> activeSession = session.lock();
    activeSession->write(msg);
}

