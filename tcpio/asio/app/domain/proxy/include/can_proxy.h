#ifndef F8786CC9_59FA_42BE_9B73_3CB1FA60A4F3
#define F8786CC9_59FA_42BE_9B73_3CB1FA60A4F3

#include "iproxy.h"

class CanProxy : public IProxy
{
public:
    CanProxy(asio::io_context& ioService, short port);
    ~CanProxy() = default;

private:
    virtual void Init() override;
    virtual void DoClientWrite(const char* buf , const uint16_t len) {}
    virtual void DoServerWrite(const char* buf , const uint16_t len);

private:
    void KeepAlive(const uint32_t interval);
    void WriteServerAliveMsg();

private:
    AdasAsioTcpServer<DefaultChatMessage, DefaultChatMessage> server;
};

#endif /* F8786CC9_59FA_42BE_9B73_3CB1FA60A4F3 */
