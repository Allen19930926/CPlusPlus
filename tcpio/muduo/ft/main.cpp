#include <iostream>
#include "adas_tcp_client.h"
#include "adas_tcp_server.h"
#include <thread>
#include "event_dispatcher.h"
#include "event_queue.h"

using namespace muduo;
using namespace muduo::net;


void StartIoThread()
{
  EventLoop loop;
  InetAddress listenAddr(7500);
  AdasTcpServer server(&loop, listenAddr);
  server.start();

  InetAddress serverAddr("192.168.233.137", 9000);
  AdasTcpClient client(&loop, serverAddr);
  client.connect();
  loop.loop();
}

void StartEventThread()
{ 
  while(true)
  {
    // EventDispatcher::ProcessMessage();
  }

}

int main(int argc, char* argv[])
{
  LOG_INFO << "pid = " << getpid();
  std::thread io(StartIoThread);
  std::thread event(StartEventThread);
  event.detach();
  io.join();
  return 0;
}
