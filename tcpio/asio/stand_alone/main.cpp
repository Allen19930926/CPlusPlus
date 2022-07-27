#include <iostream>
#include <thread>
#include "event_dispatcher.h"
#include "event_queue.h"
#include "proxy_interface.h"
#include "proxy_repository.h"
#include <vector>
#include <memory>
#include "camera_fusion_algorithm.h"
#include "communication_interface.h"

void DoPeriodTask(asio::steady_timer* timer, uint32_t interval)
{
    std::cout << " camera fusion " << std::endl;
    CameraFusionAlgo::ExecuteCameraDataFusion();
    timer->expires_after(std::chrono::milliseconds(interval));
    timer->async_wait(std::bind(&DoPeriodTask, timer, interval));
}

void StartIoThread()
{
    asio::io_context io;
    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<CanProxy>(io, 9000));
    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<GSentryProxy>(io, "127.0.0.1", "7500"));
    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<HmiProxy>(io, "127.0.0.1", "5000", PAD_SERVER_PORT));
    CDD_FUSION_PROXY_REPO.Start();

    std::shared_ptr<IProxy> ptr;
    if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::V2X, ptr))
    {
        // ptr->SetPeriodWriteTask(IProxy::TCP_CLIENT, 2000, "gSentry proxy period write");
    }

    io.run();
}

void StartEventThread()
{ 
    while(true)
    {
        EventMessage&& msg = CDD_FUSION_EVENT_QUEUE.wait_and_get_front();
        // std::cout << "after construct" << std::endl;
        EventDispatcher::ProcessMessage(msg);
    }
}
void StartPeriodThread()
{
    asio::io_context io;
    uint32_t interval = 3000;
    asio::steady_timer timer(io, std::chrono::milliseconds(interval));
    timer.async_wait(std::bind(&DoPeriodTask, &timer, interval));
    io.run();
}

int main(int argc, char* argv[])
{
    try
    {
        std::thread io(StartIoThread);
        std::thread event(StartEventThread);
        std::thread period(StartPeriodThread);
        start_j3_communicaiton();
        event.detach();
        period.detach();
        io.join();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}
