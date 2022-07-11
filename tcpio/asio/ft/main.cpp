#include <iostream>
#include <thread>
#include "event_dispatcher.h"
#include "event_queue.h"
#include "adas_asio_tcp_server.h"
#include "adas_asio_tcp_client.h"

void StartIoThread()
{
    asio::io_context io;
    AdasAsioTcpServer server(io, 9000);

    tcp::resolver resolver(io);
    auto endPoints = resolver.resolve("127.0.0.1", "7500");
    AdasAsioTcpClient client(io, endPoints);
    io.run();

    while (client.IsConnected())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    client.close();

}

void StartEventThread()
{ 
    while(true)
    {
        EventDispatcher::ProcessMessage();
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
