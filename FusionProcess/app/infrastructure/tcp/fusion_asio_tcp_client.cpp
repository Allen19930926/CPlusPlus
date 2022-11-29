#include "fusion_asio_tcp_client.h"
#include "infrastructure/common/log_debug.h"
#include "glog/logging.h"

namespace
{
    DefaultChatMessage aliveMsg("keep alive");
}

FusionAsioTcpClient::FusionAsioTcpClient(asio::io_context& io_context, std::string ipAddr, std::string port)
    : io_context_(io_context), socket_(io_context), reconnectTimer(io_context), aliveTimer(io_context),
    isConnected(false), ip(ipAddr), port_(port), reconnectCount(0)
{
}

void FusionAsioTcpClient::close()
{
    asio::post(io_context_, [this]() {socket_.close(); });
}

void FusionAsioTcpClient::start()
{
    do_connect();
    keep_alive();
}

void FusionAsioTcpClient::set_read_cb(ReadCB read_cb)
{
    read_call_back = read_cb;
}

void FusionAsioTcpClient::keep_alive()
{
    write(aliveMsg);
    aliveTimer.expires_after(std::chrono::seconds(5));
    aliveTimer.async_wait(std::bind(&FusionAsioTcpClient::keep_alive, this));
}

void FusionAsioTcpClient::do_connect()
{
    tcp::resolver resolver(io_context_);
    auto endpoints = resolver.resolve(ip, port_);

    asio::async_connect(socket_, endpoints,
        [this](std::error_code ec, tcp::endpoint)
        {
            // 防止多线程进行多次重连
            if (isConnected.load())
            {
                return;
            }

            if (!ec)
            {
                doReadHeader();
                isConnected.store(true);
                reconnectCount = 0;
                reconnectTimer.cancel();
                LOG(INFO) << "connected!!  IP:" << socket_.remote_endpoint().address() << "  Port: " << socket_.remote_endpoint().port();
            }
            else
            {
                if (reconnectCount > 20)
                {
                    LOG(INFO) << "try connection 20 times, stop retry!!";
                    return;
                }
                LOG(INFO) << "connection failed, bs will retry in 30s";
                reconnectTimer.expires_after(std::chrono::seconds(30));
                reconnectTimer.async_wait(std::bind(&FusionAsioTcpClient::do_connect, this));
                reconnectCount++;
            }
        });
}

void FusionAsioTcpClient::doReadHeader()
{
    asio::async_read(socket_, asio::buffer(readMsg.Data(), FusionChatMessage::HeaderLength),
        [this](std::error_code ec, std::size_t length)
        {
            CDebugFun::PrintBuf(reinterpret_cast<uint8_t*>(readMsg.Data()), FusionChatMessage::HeaderLength);
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
                read_call_back(std::move(readMsg));
                doReadHeader();
            }
        });
}

void FusionAsioTcpClient::write(const DefaultChatMessage& msg)
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

void FusionAsioTcpClient::do_write()
{
    // 连接状态：通过异步写操作，写入成功，则继续写入写消息队列中的内容；
    //                       写入失败，且失败原因是broken_pipe或connection_aborted，则重置连接状态为false，并进行重连。
    //                       无论写入成功失败，均丢弃该条消息，且继续尝试写入队列其他信息，防止消息积压。

    // 未连接状态： 丢弃该条消息，且继续尝试写入队列其他信息，防止消息积压。
    if (isConnected.load())
    {
        socket_.async_write_some(
            asio::buffer(writeMsgs.front().Data(), writeMsgs.front().BodyLength()),
            [this](std::error_code ec, std::size_t /*length*/)
            {
                if (ec.value() == EPIPE || ec.value() == ECONNABORTED)
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
        return;
    }
    writeMsgs.pop_front();
    if (!writeMsgs.empty())
    {
        do_write();
    }
}
