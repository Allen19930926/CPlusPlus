#include "fusion_asio_tcp_client.h"


FusionAsioTcpClient::FusionAsioTcpClient(asio::io_context& io_context, std::string ipAddr, std::string port)
    : io_context_(io_context), socket_(io_context), timer(io_context)
{
    tcp::resolver resolver(io_context_);
    endpoints_ = resolver.resolve(ipAddr, port);
}

void FusionAsioTcpClient::close()
{
    asio::post(io_context_,[this](){socket_.close();});
}

void FusionAsioTcpClient::start()
{
    do_connect(endpoints_);
}

void FusionAsioTcpClient::do_connect(const tcp::resolver::results_type& endpoints)
{
    asio::async_connect(socket_, endpoints,
        [this](std::error_code ec, tcp::endpoint)
        {
            if (!ec)
            {
                timer.cancel();
                doReadHeader();
            }
            else
            {
                // std::cout << "Connect error(line:80) : " << ec.message() << ", trying to reconnecting in every 1s" << std::endl;
                timer.expires_after(std::chrono::seconds(1));
                timer.async_wait(std::bind(&FusionAsioTcpClient::do_connect, this, endpoints_));
            }
        });
}

void FusionAsioTcpClient::doReadHeader()
{
    asio::async_read(socket_, asio::buffer(readMsg.Data(), FusionChatMessage::HeaderLength),
            [this](std::error_code ec, std::size_t length)
            {
                if (!ec && readMsg.DecodeHeader())
                {
                    doReadBody();
                }
            });
}

void FusionAsioTcpClient::doReadBody()
{
    asio::async_read(socket_, asio::buffer(readMsg.Body(), readMsg.BodyLength()),
            [this](std::error_code ec, std::size_t length)
            {
                if (!ec)
                {
                    // call fuse_frame func
                    doReadHeader();
                }
            });
}
