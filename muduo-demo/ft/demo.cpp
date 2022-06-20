#include <iostream>
#include "AdasTcpClient.h"
#include "AdasTcpServer.h"

int main(int argc, char* argv[])
{
  LOG_INFO << "pid = " << getpid();
  EventLoop loop;

  if (argc <= 2)
  {
    InetAddress listenAddr(7500);
    AdasTcpServer client(&loop, listenAddr);
    client.start();
    loop.loop();
  }
  else
  {
    uint16_t port = static_cast<uint16_t>(atoi(argv[2]));
    InetAddress serverAddr(argv[1], port);

    std::cout << argv[1] << ": " << argv[2] << std::endl;

    AdasTcpClient client(&loop, serverAddr);
    client.connect();
    loop.loop();
  }
  return 0;
}
