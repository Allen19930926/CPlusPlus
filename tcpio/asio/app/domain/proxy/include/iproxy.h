#ifndef B7C663B3_1E76_4947_8AE7_C977E5A08220
#define B7C663B3_1E76_4947_8AE7_C977E5A08220

#include "adas_asio_tcp_server.h"
#include "adas_asio_tcp_client.h"
#include <iostream>
#include "event_msg.h"
#include <string>

class IProxy
{
public:
    enum ConnectType {TCP_CLIENT, TCP_SERVER};
    using PeriodTimer = std::shared_ptr<asio::steady_timer>;

protected:
    IProxy(asio::io_context& io);

public:
    void Start();
    void SetPeriodWriteTask(const ConnectType type, const uint32_t interval, std::string msg);

private:
    virtual void Init() = 0;
    virtual void DoPeriodClientWriteTask(PeriodTimer timer ,const uint32_t interval, std::string msg) = 0;
    virtual void DoPeriodServerWriteTask(PeriodTimer timer ,const uint32_t interval, std::string msg) = 0;

protected:
    asio::io_context& io_context_;
    std::vector< std::shared_ptr<asio::steady_timer>> timers;
};

#endif /* B7C663B3_1E76_4947_8AE7_C977E5A08220 */
