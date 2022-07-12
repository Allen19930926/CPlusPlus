#ifndef F8786CC9_59FA_42BE_9B73_3CB1FA60A4F3
#define F8786CC9_59FA_42BE_9B73_3CB1FA60A4F3

#include "iproxy.h"

class CanProxy : public IProxy
{
public:
    CanProxy(asio::io_context& ioService, short port) : server(ioService, MsgType::CAN, port) {}
    ~CanProxy() = default;
    virtual void Start() override { server.start(); }
    void SetPeriodWriteTask(const uint32_t interval) { server.SetPeriodWriteTask(interval, keepAliveMsg); }
private:
    AdasAsioTcpServer server;
    const std::string keepAliveMsg = "CAN(server) keep alive!! CAN(server) keep alive!! ";
};

#endif /* F8786CC9_59FA_42BE_9B73_3CB1FA60A4F3 */
