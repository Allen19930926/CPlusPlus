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
    void KeepAlive(const uint32_t interval);
    virtual void DoPeriodServerWriteTask(PeriodTimer timer ,const uint32_t interval, std::string msg) override;
    virtual void DoPeriodClientWriteTask(PeriodTimer timer ,const uint32_t interval, std::string msg) override {}
    virtual void DoWrite(const char* buf , const uint16_t len) {}

private:
    AdasAsioTcpServer<BasciChatMessage, BasciChatMessage> server;
};

#endif /* F8786CC9_59FA_42BE_9B73_3CB1FA60A4F3 */
