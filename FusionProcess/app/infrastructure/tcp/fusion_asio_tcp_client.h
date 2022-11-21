#ifndef C5CF927E_9DE0_47DA_85F1_E5A9DEE64D22
#define C5CF927E_9DE0_47DA_85F1_E5A9DEE64D22

#include "asio.hpp"
#include "asio/steady_timer.hpp"
#include <string>
#include <functional>
#include <queue>
#include "application/fusion_system.h"
#include "infrastructure/common/fusion_chat_message.h"
#include "infrastructure/common/default_chat_message.h"

using asio::ip::tcp;

class FusionAsioTcpClient
{
public:
    FusionAsioTcpClient(asio::io_context& io_context, std::string ipAddr, std::string port);
    void close();
    void start();

private:
    void keep_alive();
    void do_connect();
    void doReadHeader();
    void doReadBody();
    void write(const DefaultChatMessage& msg);
    void do_write();

private:
    asio::io_context& io_context_;
    tcp::socket socket_;
    asio::steady_timer reconnectTimer;
    asio::steady_timer aliveTimer;
    FusionChatMessage readMsg;
    std::deque<DefaultChatMessage> writeMsgs;
    std::atomic_bool isConnected;
    std::string ip;
    std::string port_;
    uint32_t reconnectCount;
    FusionSystem fuse_system;
};

#endif /* C5CF927E_9DE0_47DA_85F1_E5A9DEE64D22 */
