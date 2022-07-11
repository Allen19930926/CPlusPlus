#include "adas_asio_tcp_client.h"
#include <functional>
#include <iostream>
#include "v2x_adas_event_macro.h"
#include "event_dispatcher.h"
#include "v2x_data_struct.h"
#include "cdd_fusion.h"
#include "event_queue.h"

using namespace std::placeholders;
using namespace V2X;

AdasAsioTcpClient::AdasAsioTcpClient(asio::io_context& io_context,
        const tcp::resolver::results_type& endpoints)
        : isConnected_(false), io_context_(io_context),
        socket_(io_context), deadline_(io_context)
{
    do_connect(endpoints);
}


void AdasAsioTcpClient::close()
{
    asio::post(io_context_,[this](){socket_.close();});
}

void AdasAsioTcpClient::do_connect(const tcp::resolver::results_type& endpoints)
{
    endpoints_ = endpoints;
    asio::async_connect(socket_, endpoints,
        [this](std::error_code ec, tcp::endpoint)
        {
            if (!ec)
            {
                isConnected_ = true;
                deadline_.cancel();
                do_read();
            }
            else
            {
                std::cout << "Connect error: " << ec.message() << ", trying to reconnecting in every 1s" << std::endl;
                deadline_.expires_after(std::chrono::seconds(1));
                deadline_.async_wait(std::bind(&AdasAsioTcpClient::do_connect, this, endpoints_));
            }
        });
}

void AdasAsioTcpClient::do_read()
{
    socket_.async_read_some(asio::buffer(data_, max_length), std::bind(&AdasAsioTcpClient::on_read, this,_1,_2));
}

void AdasAsioTcpClient::on_read(const std::error_code & ec, size_t bytes)
{
    std::cout << "recieve msg : " << &data_[0] << std::endl;
    uint16_t headlen = sizeof(V2xAdasMsgHeader);
    if (!ec && bytes >= headlen)
    {
        // 正常读取数据后，直接将数据域添加到消息队列中
        CDD_FUSION_EVENT_QUEUE.push({0x101, data_ + headlen, static_cast<uint16_t>(bytes - headlen)});
    }
    do_read();
}
