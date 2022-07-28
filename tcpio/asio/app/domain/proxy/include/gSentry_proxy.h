#ifndef B243B601_9C74_4478_A22A_1A6A19103451
#define B243B601_9C74_4478_A22A_1A6A19103451

#include "iproxy.h"
#include <string>

class GSentryProxy : public IProxy
{
public:
    GSentryProxy(asio::io_context& ioService, std::string ipAddr, std::string port);
    ~GSentryProxy() = default;

    // for demo
    void WritePeriodData(const uint32_t interval);

private:
    virtual void Init() override;
    virtual void DoClientWrite(const char* buf , const uint16_t len);
    virtual void DoServerWrite(const char* buf , const uint16_t len) {}

private:
    void KeepAlive(const uint32_t interval);
    void WriteClientAliveMsg();
    void WriteData();

private:
    AdasAsioTcpClient<DefaultChatMessage, DefaultChatMessage> client;
};

#endif /* B243B601_9C74_4478_A22A_1A6A19103451 */
