#ifndef B7C663B3_1E76_4947_8AE7_C977E5A08220
#define B7C663B3_1E76_4947_8AE7_C977E5A08220

#include "adas_asio_tcp_server.hpp"
#include "adas_asio_tcp_client.hpp"
#include <iostream>
#include "event_msg.h"
#include <string>
#include "default_chat_message.h"
#include <functional>

class IProxy
{
public:
    enum ConnectType {TCP_CLIENT, TCP_SERVER};
    using PeriodTimer = std::shared_ptr<asio::steady_timer>;

protected:
    IProxy(asio::io_context& io, MsgType type_);
    using PeriodWriteCallBack = std::function<void(void)>;

public:
    void Start();
    void Write(const ConnectType type, const char* buf, const uint16_t len);
    void SetPeriodTask(const uint32_t interval, PeriodWriteCallBack cb);
    bool IsSpecificProxyType(MsgType type_) {return type_ == type;}

private:
    virtual void Init() = 0;
    virtual void DoClientWrite(const char* buf , const uint16_t len) = 0;
    virtual void DoServerWrite(const char* buf , const uint16_t len) = 0;
    void DoPeriodWriteTask(PeriodTimer timer ,const uint32_t interval, PeriodWriteCallBack cb);

protected:
    asio::io_context& io_context_;
    std::vector< std::shared_ptr<asio::steady_timer>> timers;
    const MsgType type;
};

#endif /* B7C663B3_1E76_4947_8AE7_C977E5A08220 */
