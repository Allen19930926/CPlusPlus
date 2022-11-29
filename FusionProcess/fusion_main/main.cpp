#include "application/data_reciever/data_reciever.h"
#include "application/fuse_system/fusion_system.h"
#include "glog/logging.h"
#include <thread>

FusionSystem sys;

void DoPeriodFusionTask(asio::steady_timer* timer, uint32_t interval)
{
    sys.Fuse();
    timer->expires_after(std::chrono::milliseconds(interval));
    timer->async_wait(std::bind(&DoPeriodFusionTask, timer, interval));
}

void StartIoThread()
{
    asio::io_context io_context;
    DataReciever  data_reciever(io_context, "192.168.198.131", "35243");
    data_reciever.Start();
    io_context.run();
}


void StartPeriodThread()
{
    asio::io_context io;

    // 创建定时器，周期发送data数据
    uint32_t interval = 50;
    asio::steady_timer timer(io, std::chrono::milliseconds(interval));
    timer.async_wait(std::bind(&DoPeriodFusionTask, &timer, interval));
    io.run();
}

int main(int argc, char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    try
    {
        std::thread io(StartIoThread);
        std::thread period(StartPeriodThread);
        io.detach();
        period.join();
    }
    catch(const std::exception& e)
    {
        LOG(ERROR) << e.what() << '\n';
    }
    
    google::ShutdownGoogleLogging();
    return 0;
}
