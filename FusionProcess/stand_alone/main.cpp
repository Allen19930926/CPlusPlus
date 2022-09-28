#include "fusion_asio_tcp_client.h"



int main(int argc, char* argv[])
{
    asio::io_context io_context;
    FusionAsioTcpClient  client(io_context, "192.168.233.178", "35243");
    client.start();
    io_context.run();
    return 0;
}
