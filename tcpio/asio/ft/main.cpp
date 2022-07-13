#include <iostream>
#include <thread>
#include "event_dispatcher.h"
#include "event_queue.h"
#include "proxy_interface.h"
#include <vector>
#include <memory>

void StartIoThread()
{
    asio::io_context io;
    std::vector<std::unique_ptr<IProxy>> proxyVec;
    proxyVec.push_back(std::make_unique<CanProxy>(io, 9000));
    proxyVec.push_back(std::make_unique<GSentryProxy>(io, "127.0.0.1", "7500"));
    // proxyVec.push_back(std::make_unique<HmiProxy>(io, "127.0.0.1", "7500", 9000));

    for(auto& it : proxyVec)
    {
        it->SetPeriodWriteTask(1000);
        it->Start();
    }

    io.run();
}

void StartEventThread()
{ 
    while(true)
    {
        EventMessage&& msg = CDD_FUSION_EVENT_QUEUE.wait_and_get_front();
        EventDispatcher::ProcessMessage(msg);
    }
}

int main(int argc, char* argv[])
{
    try
    {
        std::thread io(StartIoThread);
        std::thread event(StartEventThread);
        event.detach();
        io.join();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return 0;
}
