#include "fusion_asio_tcp_client.h"
#include "glog/logging.h"


int main(int argc, char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    asio::io_context io_context;
    FusionAsioTcpClient  client(io_context, "192.168.198.131", "35243");
    client.start();
    io_context.run();
    google::ShutdownGoogleLogging();
    return 0;
}
