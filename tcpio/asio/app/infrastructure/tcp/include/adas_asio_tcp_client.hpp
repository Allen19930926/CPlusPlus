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
#include <glog/logging.h>

using asio::ip::tcp;

template<typename ReadMsg, typename WriteMsg>
class AdasAsioTcpClient
{
public:
    AdasAsioTcpClient(asio::io_context& io_context, MsgType type, std::string ipAddr, std::string port)
    : io_context_(io_context), socket_(io_context), timer(io_context), msgType(type), isConnected(false),
      ip(ipAddr), port_(port), reconnect_count(0)
    {
    }

    void close()
    {
        asio::post(io_context_,[this](){socket_.close();});
    }

    void start()
    {
        do_connect();
    }

    void write(const WriteMsg& msg)
    {
        asio::post(io_context_, [this, msg]()
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
    void do_connect()
    {
        tcp::resolver resolver(io_context_);
        auto endpoints = resolver.resolve(ip, port_);

        asio::async_connect(socket_, endpoints,
            [this](std::error_code ec, tcp::endpoint)
            {
                // 防止多线程进行多次重连
                if (isConnected.load())
                {
                    return ;
                }

                if (!ec)
                {
                    doReadHeader();
                    isConnected.store(true);
                    reconnect_count = 0;
                    timer.cancel();
                    LOG(INFO) << "connected!!  IP:" << socket_.remote_endpoint().address() << "  Port: " << socket_.remote_endpoint().port() << " MsgType: " << int(msgType);
                }
                else
                {
                    if (reconnect_count > 30)
                    {
                        LOG(INFO) << "try connection 30 times, stop retry!!" << " MsgType: " << int(msgType);
                        return ;
                    }
                    LOG(INFO) << "connection " << ip << " failed, bs will retry in 10s" << " MsgType: " << int(msgType);
                    timer.expires_after(std::chrono::seconds(10));
                    timer.async_wait(std::bind(&AdasAsioTcpClient::do_connect, this));
                    reconnect_count++;
                }
            });
    }

    void do_write()
    {
        // 连接状态：通过异步写操作，写入成功，则继续写入写消息队列中的内容；
		//				  写入失败，且失败原因是broken_pipe或connection_aborted，则重置连接状态为false，并进行重连。
		//				  无论写入成功失败，均丢弃该条消息，且继续尝试写入队列其他信息，防止消息积压。
						  
        // 未连接状态： 丢弃该条消息，且继续尝试写入队列其他信息，防止消息积压。
        if (isConnected.load())
        {
            //LOG(INFO) << "writeMsgs.front().Data(): " << writeMsgs.front().Body() << "  writeMsgs.front().BodyLength():" << writeMsgs.front().BodyLength();
            socket_.async_write_some(
                asio::buffer(writeMsgs.front().Data(), writeMsgs.front().Length()),
                [this](std::error_code ec, std::size_t /*length*/)
                {
                    if(ec.value() == EPIPE  || ec.value() == ECONNABORTED)
                    {
                        LOG(INFO) << "write fail reason: " << ec.message();
                        isConnected.store(false);
                        socket_.close();
                        do_connect();
                    }
                    writeMsgs.pop_front();
                    if (!writeMsgs.empty())
                    {
                        do_write();
                    }
                    
                });
            return ;
        }
        writeMsgs.pop_front();
        if (!writeMsgs.empty())
        {
            do_write();
        }
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
                        // std::cout << "client read readMsg.Body(): " << readMsg.Body() << "  readMsg.BodyLength():" << readMsg.BodyLength() << std::endl;
                        if (msgType != MsgType::V2X)
                        {
                            CDD_FUSION_EVENT_QUEUE.push({msgType, readMsg.Body(), static_cast<uint16_t>(readMsg.BodyLength())});
                        }
                        else
                        {
                            CDD_FUSION_EVENT_QUEUE.push({msgType, readMsg.Data(), static_cast<uint16_t>(readMsg.Length())});
                        }
                        
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
    MsgType msgType;
    std::atomic_bool isConnected;
    std::string ip;
    std::string port_;
    uint32_t reconnect_count;
};


#endif /* B24BA3E3_14EC_47C8_95B7_EC70F30AAA60 */
