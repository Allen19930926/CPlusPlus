#ifndef C5CF927E_9DE0_47DA_85F1_E5A9DEE64D22
#define C5CF927E_9DE0_47DA_85F1_E5A9DEE64D22

#include "asio.hpp"
#include "asio/steady_timer.hpp"
#include <string>
#include <functional>
#include "fusion_chat_message.h"

using asio::ip::tcp;

class FusionAsioTcpClient
{
public:
    FusionAsioTcpClient(asio::io_context& io_context, std::string ipAddr, std::string port);
    void close();
    void start();

private:
    void do_connect(const tcp::resolver::results_type& endpoints);
    void doReadHeader();
    void doReadBody();

private:
    asio::io_context& io_context_;
    tcp::socket socket_;
    asio::steady_timer timer;
    tcp::resolver::results_type endpoints_;
    FusionChatMessage readMsg;
};

#endif /* C5CF927E_9DE0_47DA_85F1_E5A9DEE64D22 */
