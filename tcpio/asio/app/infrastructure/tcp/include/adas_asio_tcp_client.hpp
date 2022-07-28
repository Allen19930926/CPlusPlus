#ifndef B24BA3E3_14EC_47C8_95B7_EC70F30AAA60
#define B24BA3E3_14EC_47C8_95B7_EC70F30AAA60

#include "asio.hpp"
#include "asio/steady_timer.hpp"
#include <string>
#include <functional>
#include "event_msg.h"
#include <atomic>
#include <iostream>
#include "v2x_adas_event_macro.h"
#include "event_dispatcher.h"
#include "v2x_data_struct.h"
#include "cdd_fusion.h"
#include "event_queue.h"

using asio::ip::tcp;

template<typename ReadMsg, typename WriteMsg>
class AdasAsioTcpClient
{
public:
    AdasAsioTcpClient(asio::io_context& io_context, MsgType type, std::string ipAddr, std::string port)
    : io_context_(io_context), socket_(io_context), timer(io_context), msgType(type), isConnected(false)
    {
        tcp::resolver resolver(io_context_);
        endpoints_ = resolver.resolve(ipAddr, port);
    }

    void close()
    {
        asio::post(io_context_,[this](){socket_.close();});
    }

    void start()
    {
        do_connect(endpoints_);
    }

    void write(const WriteMsg& msg)
    {
        asio::post(io_context_, [this, msg]()
        {
            if (!isConnected.load())
            {
                return ;
            }

            bool write_in_progress = !writeMsgs.empty();
            writeMsgs.push_back(msg);
            if (!write_in_progress)
            {
                do_write();
            }
        });
    }

private:
    void do_connect(const tcp::resolver::results_type& endpoints)
    {
        asio::async_connect(socket_, endpoints,
            [this](std::error_code ec, tcp::endpoint)
            {
                if (!ec)
                {
                    timer.cancel();
                    doReadHeader();
                    isConnected.store(true);
                }
                else
                {
                    // std::cout << "Connect error(line:80) : " << ec.message() << ", trying to reconnecting in every 1s" << std::endl;
                    timer.expires_after(std::chrono::seconds(1));
                    timer.async_wait(std::bind(&AdasAsioTcpClient::do_connect, this, endpoints_));
                }
            });
    }

    void do_write()
    {
        socket_.async_write_some(
            asio::buffer(writeMsgs.front().Data(), writeMsgs.front().BodyLength()),
            [this](std::error_code ec, std::size_t /*length*/)
            {
                if (!ec)
                {
                    std::cout << "client write : " << writeMsgs.front().Data() << std::endl;
                    writeMsgs.pop_front();
                    if (!writeMsgs.empty())
                    {
                        do_write();
                    }
                }
            });
    }

    void doReadHeader()
    {
        asio::async_read(socket_, asio::buffer(readMsg.Data(), ReadMsg::HeaderLength),
                [this](std::error_code ec, std::size_t length)
                {
                    if (!ec && readMsg.DecodeHeader())
                    {
                        doReadBody();
                    }
                });
    }

    void doReadBody()
    {
        asio::async_read(socket_, asio::buffer(readMsg.Body(), readMsg.BodyLength()),
                [this](std::error_code ec, std::size_t length)
                {
                    if (!ec)
                    {
                        // std::cout << "client read : " << readMsg.Body() << std::endl;
                        CDD_FUSION_EVENT_QUEUE.push({msgType, readMsg.Body(), static_cast<uint16_t>(readMsg.BodyLength())});
                        doReadHeader();
                    }
                });
    }

private:
    asio::io_context& io_context_;
    tcp::socket socket_;
    ReadMsg readMsg;
    std::deque<WriteMsg> writeMsgs;
    asio::steady_timer timer;
    tcp::resolver::results_type endpoints_;
    MsgType msgType;
    std::atomic_bool isConnected;
};


#endif /* B24BA3E3_14EC_47C8_95B7_EC70F30AAA60 */
