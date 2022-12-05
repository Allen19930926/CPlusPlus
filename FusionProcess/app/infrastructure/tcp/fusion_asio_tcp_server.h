#ifndef E5779DA7_190F_4C14_8F98_74413CAFD006
#define E5779DA7_190F_4C14_8F98_74413CAFD006

#include "asio.hpp"
#include "asio/steady_timer.hpp"
#include <string>
#include <functional>
#include <queue>
#include "infrastructure/common/fusion_chat_message.h"
#include "infrastructure/common/default_chat_message.h"

using asio::ip::tcp;
using ReadCB =  std::function<void(FusionChatMessage)>;

class FusionAsioTcpSession : public std::enable_shared_from_this<FusionAsioTcpSession>
{
public:
    FusionAsioTcpSession(asio::io_context& io, tcp::socket socket, asio::io_context& io_context, ReadCB cb);
    void start();
    void write(const DefaultChatMessage& msg);
private:
    void doReadHeader();
    void doReadBody();
    void do_write();

private:
    asio::io_context& io_context_;
    tcp::socket socket_;
	FusionChatMessage readMsg;
	std::deque<DefaultChatMessage> writeMsgs;
    ReadCB read_call_back;
};


class FusionAsioTcpServer
{
public:
    FusionAsioTcpServer(asio::io_context& io_context, short port, ReadCB cb);
    void start();
    void write(DefaultChatMessage msg);

private:
    asio::io_context& io_context_;
    tcp::acceptor acceptor_;
    std::weak_ptr<FusionAsioTcpSession> session;
    ReadCB read_call_back;
};


#endif /* E5779DA7_190F_4C14_8F98_74413CAFD006 */
