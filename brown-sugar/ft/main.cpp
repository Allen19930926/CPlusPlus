#include "gtest/gtest.h"
#include "glog/logging.h"
#include <iostream>
#include <thread>
#include "event_dispatcher.h"
#include "event_queue.h"
#include "hmi_proxy.h"
#include "ipc_proxy.h"
#include "cds_proxy.h"
#include "xds_proxy.h"
#include "proxy_interface.h"
#include "proxy_repository.h"
#include <memory>
#include "camera_fusion_algorithm.h"
#include "hmi_udp_addressing_server.hpp"


#define XDS_LISTEN_PORT 35559
#define HMI_LISTEN_PORT 35553
#define CDS_LISTEN_PORT 34567

std::atomic_bool isRunning;


void StartIoThread()
{
    asio::io_context io;
    HmiUdpAddressingServer addressServer(io, HMI_LISTEN_PORT);
    addressServer.start();

    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<CanProxy>(io, 9000));
    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<GSentryProxy>(io, "192.168.1.199", "50600"));
    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<HmiProxy>(io, "127.0.0.1", "5000", HMI_LISTEN_PORT));
    // CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<IpcProxy>(io));
    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<XdsProxy>(io, XDS_LISTEN_PORT));
    CDD_FUSION_PROXY_REPO.AddProxy(std::make_shared<CdsProxy>(io, CDS_LISTEN_PORT));
    CDD_FUSION_PROXY_REPO.Start();

    io.run();
}

void StartEventThread()
{ 
    while(isRunning.load())
    {
        EventMessage&& msg = CDD_FUSION_EVENT_QUEUE.wait_and_get_front();
        // std::cout << "after construct" << std::endl;
        EventDispatcher::ProcessMessage(msg);
    }
}

void StartLogConfig()
{
    FLAGS_minloglevel = 3;
    google::InitGoogleLogging("ftlog");
}

int main(int argc, char* argv[])
{
    try
    {
        isRunning.store(true);
        StartLogConfig();
        std::thread io(StartIoThread);
        std::thread event(StartEventThread);
        event.detach();
        io.detach();
        testing::InitGoogleTest(&argc, argv);
        RUN_ALL_TESTS();
        isRunning.store(false);
        CDD_FUSION_EVENT_QUEUE.push({MsgType::V2X, nullptr, 0});
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}
