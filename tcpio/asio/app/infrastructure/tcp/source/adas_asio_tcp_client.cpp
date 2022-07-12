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

using std::string;

AdasAsioTcpClient::AdasAsioTcpClient(asio::io_context& io_context, MsgType type, std::string ipAddr, std::string port)
        : io_context_(io_context), socket_(io_context), deadline_(io_context), msgType(type), aliveInterval(0xFFFFFFFF)
{
    tcp::resolver resolver(io_context_);
    endpoints_ = resolver.resolve(ipAddr, port);
}

void AdasAsioTcpClient::start()
{
    do_connect(endpoints_);
}

void AdasAsioTcpClient::close()
{
    asio::post(io_context_,[this](){socket_.close();});
}

void AdasAsioTcpClient::SetPeriodWriteTask(const uint32_t interval, string msg)
{
    deadline_.cancel();
    aliveInterval = interval;
    aliveMsg = msg;
}

void AdasAsioTcpClient::do_write(string msg)
{
    socket_.async_write_some(asio::buffer(msg.data(), msg.length()),
        [this](std::error_code ec, std::size_t length)
        {
            if (!ec) 
            {
                // to do
            }
        });
}

void AdasAsioTcpClient::do_period_write(const uint32_t interval, std::string msg)
{
    socket_.async_write_some(asio::buffer(msg.data(), msg.length() + 1),
        [this, interval, msg](std::error_code ec, std::size_t length)
        {
            // std::cout << ec.message() << std::endl;
            if (!ec) 
            {
                std::cout << "client period write(line:62) " << msg << std::endl;
                deadline_.expires_after(std::chrono::milliseconds(interval));
                deadline_.async_wait(std::bind(&AdasAsioTcpClient::do_period_write, this, interval, msg));
            }
        });
}

void AdasAsioTcpClient::do_connect(const tcp::resolver::results_type& endpoints)
{
    asio::async_connect(socket_, endpoints,
        [this](std::error_code ec, tcp::endpoint)
        {
            if (!ec)
            {
                deadline_.cancel();
                do_read();
                if (aliveInterval <= 10800)
                {
                    do_period_write(aliveInterval, aliveMsg);
                }
            }
            else
            {
                // std::cout << "Connect error(line:80) : " << ec.message() << ", trying to reconnecting in every 1s" << std::endl;
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
    // std::cout << "recieve msg(line:94, " << bytes << "bytes ) : " << &data_[0] << std::endl;
    if (!ec)
    {
        CDD_FUSION_EVENT_QUEUE.push({msgType, data_, static_cast<uint16_t>(bytes)});
    }
    do_read();
}
