#include "iproxy.h"

using std::string;

IProxy::IProxy(asio::io_context& io) : io_context_(io)
{

}

void IProxy::Start()
{
    Init();
}

void IProxy::SetPeriodWriteTask(const ConnectType type, const uint32_t interval, string msg)
{
    if (timers.size() > 10)
    {
        // 软限位，每个proxy目前设置10个周期定时器是满足使用需求的
        return ;
    }
    PeriodTimer timer = std::make_shared<asio::steady_timer>(io_context_);
    timers.push_back(timer);
    if (type == TCP_CLIENT)
    {
        return DoPeriodClientWriteTask(timer, interval, msg);
    }
    return DoPeriodServerWriteTask(timer, interval, msg);
}

