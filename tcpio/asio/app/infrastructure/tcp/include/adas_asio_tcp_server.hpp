#ifndef CD72DBDC_366D_4F08_B091_0522E40C3FE7
#define CD72DBDC_366D_4F08_B091_0522E40C3FE7

#include "asio.hpp"
#include "asio/steady_timer.hpp"
#include "event_msg.h"
#include <string>
#include <iostream>
// #include "v2x_adas_event_macro.h"
// #include "event_dispatcher.h"
// #include "v2x_data_struct.h"
// #include "cdd_fusion.h"
#include "event_queue.h"

using asio::ip::tcp;

template<typename ReadMsg, typename WriteMsg>
class AdasAsioTcpSession : public std::enable_shared_from_this<AdasAsioTcpSession<ReadMsg, WriteMsg>>
{
public:
    AdasAsioTcpSession(asio::io_context& io, tcp::socket socket, asio::io_context& io_context, MsgType type)
    : io_context_(io), socket_(std::move(socket)), msgType(type)
    {
        socket_.set_option(asio::ip::tcp::socket::reuse_address(true));
    }
    void start()
    {
        doReadHeader();
    }

    void write(const WriteMsg& msg)
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

private:
    void doReadHeader()
    {
        auto self(this->shared_from_this());
        asio::async_read(socket_, asio::buffer(readMsg.Data(), ReadMsg::HeaderLength),
            [this, self](std::error_code ec, std::size_t /*length*/)
            {
                if (!ec && readMsg.DecodeHeader())
                {
                    doReadBody();
                }
        });
    }

    void doReadBody()
    {
        auto self(this->shared_from_this());
        asio::async_read(socket_, asio::buffer(readMsg.Body(), readMsg.BodyLength()),
            [this, self](std::error_code ec, std::size_t /*length*/)
            {
            if (!ec)
            {
                std::cout << "server read : " << readMsg.Body() << std::endl;
                CDD_FUSION_EVENT_QUEUE.push({ msgType, readMsg.Body(), static_cast<uint16_t>(readMsg.BodyLength())});
                doReadHeader();
            }
        });
    }

    void do_write()
    {
        auto self(this->shared_from_this());
        asio::async_write(socket_, asio::buffer(writeMsgs.front().Data(), writeMsgs.front().Length()),
            [this, self](std::error_code ec, std::size_t /*length*/)
            {
                if (!ec)
                {
                    std::cout << "server write : " << writeMsgs.front().Data() << std::endl;
                    writeMsgs.pop_front();
                    if (!writeMsgs.empty())
                    {
                        do_write();
                    }
                }
        });
    }

private:
    asio::io_context& io_context_;
    tcp::socket socket_;
    MsgType msgType;
	ReadMsg readMsg;
	std::deque<WriteMsg> writeMsgs;
};


template<typename ReadMsg, typename WriteMsg>
class AdasAsioTcpServer
{
public:
    AdasAsioTcpServer(asio::io_context& io_context, MsgType type, short port)
    : io_context_(io_context), acceptor_(io_context, tcp::endpoint(tcp::v4(), port)), msgType(type)
    {
    }

    void start()
    {
        acceptor_.async_accept(
            [this](std::error_code ec, tcp::socket socket)
            {
                if (!ec)
                {
                    std::shared_ptr<SessionType> activeSession = 
                            std::make_shared<SessionType>(io_context_, std::move(socket), io_context_, msgType);
                    session = activeSession;
                    activeSession->start();
                }

                start();
            });
    }

    void write(std::string msg)
    {
        // 目前设计server和client是一对一，因此server设计只与最后一次成功连接的client进行写入会话
        if (session.expired())
        {
            return ;
        }
        std::shared_ptr<SessionType> activeSession = session.lock();
        activeSession->write(msg);
    }

private:
    using SessionType = AdasAsioTcpSession<ReadMsg, WriteMsg>;

private:
    asio::io_context& io_context_;
    tcp::acceptor acceptor_;
    MsgType msgType;
    std::weak_ptr<SessionType> session;
};

#endif /* CD72DBDC_366D_4F08_B091_0522E40C3FE7 */
