#include <iostream>
#include <thread>
#include "event_dispatcher.h"
//#include "event_msg.h"
#include "event_queue.h"
#include "hmi_proxy.h"
#include "ipc_proxy.h"
#include "cds_proxy.h"
#include "xds_proxy.h"
#include "proxy_interface.h"
#include "proxy_repository.h"
#include <vector>
#include <memory>
#include "camera_fusion_algorithm.h"
#include "communication_interface.h"
#include "hmi_udp_addressing_server.hpp"
#include <glog/logging.h>
#include <signal.h>

#define XDS_LISTEN_PORT 35559
#define HMI_LISTEN_PORT 35553
#define CDS_LISTEN_PORT 35551

#ifdef __x86_64__
#include "com_tcp_server.h"
#define COM_LISTEN_PORT 35558
ComTcpServer * comTcpServer = NULL;
#endif

void TerminateHandler(int s){
    exit(1); 
}

std::string getHostAddress()
{
    asio::io_service ioService;
    asio::ip::tcp::resolver::query query("www.baidu.com", "http");
    asio::ip::tcp::resolver resolver(ioService);
    asio::ip::tcp::socket soc(ioService);
    asio::ip::tcp::endpoint ep = *(resolver.resolve(query));
    soc.connect(ep);
    return soc.local_endpoint().address().to_string();
}

void DoPeriodCameraFusionTask(asio::steady_timer* timer, uint32_t interval)
{
    // std::cout << " camera fusion " << std::endl;
    CameraFusionAlgo::ExecuteCameraDataFusion();
    timer->expires_after(std::chrono::milliseconds(interval));
    timer->async_wait(std::bind(&DoPeriodCameraFusionTask, timer, interval));
}

void DoPeriodCdsTask(asio::steady_timer* timer, uint32_t interval)
{
    std::shared_ptr<IProxy> ptr;
    if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::CDS, ptr))
    {
        auto proxy(std::dynamic_pointer_cast<CdsProxy>(ptr));
        proxy->SendperiodMessage();
    }
    timer->expires_after(std::chrono::milliseconds(interval));
    timer->async_wait(std::bind(&DoPeriodCdsTask, timer, interval));
}

void DoPeriodXdsTask(asio::high_resolution_timer& timer, const std::error_code& error_code, long long t, uint32_t interval)
{
    std::shared_ptr<IProxy> ptr;
    auto current = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::XDS_DUMMY, ptr))
    {
        auto proxy(std::dynamic_pointer_cast<XdsProxy>(ptr));
        proxy->SendMessage();
    }

    timer.expires_at(timer.expiry() + std::chrono::milliseconds(interval));
    timer.async_wait([&timer, current, interval](const std::error_code& error_code) { DoPeriodXdsTask(timer, error_code, current, interval); });
}

void DoPeriodHmiTask(asio::steady_timer* timer, uint32_t interval)
{
    std::shared_ptr<IProxy> ptr;
    if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::HMI_DUMMY, ptr))
    {
        auto proxy(std::dynamic_pointer_cast<HmiProxy>(ptr));
        proxy->DoPeriodTask();
    }
    timer->expires_after(std::chrono::milliseconds(interval));
    timer->async_wait(std::bind(&DoPeriodHmiTask, timer, interval));
}

void StartIoThread()
{
    asio::io_context io;
    HmiUdpAddressingServer addressServer(io, HMI_LISTEN_PORT);
    addressServer.start();
#ifdef __x86_64__
    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<GSentryProxy>(io, getHostAddress(), "50600"));
#else
    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<GSentryProxy>(io, "192.168.1.199", "50600"));
#endif

    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<CanProxy>(io, 9000));
    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<HmiProxy>(io, "127.0.0.1", "5000", HMI_LISTEN_PORT));
    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<IpcProxy>(io));
    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<XdsProxy>(io, XDS_LISTEN_PORT));
    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<CdsProxy>(io, CDS_LISTEN_PORT));
    CDD_FUSION_PROXY_REPO.Start();
#ifdef __x86_64__
    comTcpServer = new ComTcpServer(io, COM_LISTEN_PORT);
    comTcpServer->Start();
#endif

    // std::shared_ptr<IProxy> ptr;
    // if (CDD_FUSION_PROXY_REPO.GetSpecificProxy(MsgType::V2X, ptr))
    // {
    //     std::shared_ptr<GSentryProxy> gSentryProxy = std::dynamic_pointer_cast<GSentryProxy>(ptr);
    //     gSentryProxy->WritePeriodData(2000);
    // }

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

    // 创建定时器，周期发送data数据
    uint32_t interval = 500;
    asio::steady_timer timer(io, std::chrono::milliseconds(interval));
    timer.async_wait(std::bind(&DoPeriodCameraFusionTask, &timer, interval));

    interval = 100;
    asio::high_resolution_timer  xdsTimer(io);
    xdsTimer.expires_from_now(std::chrono::milliseconds(interval));
    auto current = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    xdsTimer.async_wait([&xdsTimer, current, interval](const std::error_code& error_code) { DoPeriodXdsTask(xdsTimer, error_code, current, interval); });

    interval = 2000;
    asio::steady_timer cdsTimer(io, std::chrono::milliseconds(interval));
    cdsTimer.async_wait(std::bind(&DoPeriodCdsTask, &cdsTimer, interval));

    interval = 2000;
    asio::steady_timer hmiTimer(io, std::chrono::milliseconds(interval));
    hmiTimer.async_wait(std::bind(&DoPeriodHmiTask, &hmiTimer, interval));

    // interval = 2000;
    // asio::steady_timer cdsTimer(io, std::chrono::milliseconds(interval));
    // cdsTimer.async_wait(std::bind(&DoPeriodCdsTask, &cdsTimer, interval));
    io.run();
}

int main(int argc, char* argv[])
{
    // struct sigaction sigIntHandler;
 
    // sigIntHandler.sa_handler = TerminateHandler;
    // sigemptyset(&sigIntHandler.sa_mask);
    // sigIntHandler.sa_flags = 0;
 
    // sigaction(SIGINT, &sigIntHandler, NULL);

    fLB::FLAGS_alsologtostderr = true;
    fLB::FLAGS_log_prefix = true;
    fLI::FLAGS_logbufsecs = 1;
    google::SetLogDestination(google::GLOG_INFO,"bs.log");
    google::InitGoogleLogging(argv[0]);
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
