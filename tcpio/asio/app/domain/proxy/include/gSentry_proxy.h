#ifndef B243B601_9C74_4478_A22A_1A6A19103451
#define B243B601_9C74_4478_A22A_1A6A19103451

#include "iproxy.h"
#include <string>

class GSentryProxy : public IProxy
{
public:
    GSentryProxy(asio::io_context& ioService, std::string ipAddr, std::string port) : client(ioService, MsgType::V2X, ipAddr, port) {}
    ~GSentryProxy() = default;
    void SetPeriodWriteTask(const uint32_t interval) { client.SetPeriodWriteTask(interval, keepAliveMsg); }
    virtual void Start() override { client.start(); }

private:
    AdasAsioTcpClient client;
    const std::string keepAliveMsg = "gSentry(client) keep alive!! gSentry(client) keep alive!! ";
};

#endif /* B243B601_9C74_4478_A22A_1A6A19103451 */
