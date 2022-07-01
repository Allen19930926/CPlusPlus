#include <iostream>
#include "adas_tcp_client.h"
#include "adas_tcp_server.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include "cdd_fusion.h"
#include "event_dispatcher.h"

using namespace muduo;
using namespace muduo::net;

std::mutex mtx;
std::condition_variable cv;
std::queue<CDDFusion::EventMessage> eventQueue;

void StartIoThread()
{
  EventLoop loop;
  InetAddress listenAddr(7500);
  AdasTcpServer server(&loop, listenAddr);
  server.start();

  InetAddress serverAddr("192.168.233.136", 9000);
  AdasTcpClient client(&loop, serverAddr);
  client.connect();
  loop.loop();
}

void StartEventThread()
{ 
  std::unique_lock<std::mutex> lck(mtx);
  while(true)
  {
    cv.wait(lck, []{return !eventQueue.empty();});
    /* to process event */
    EventDispatcher::ProcessMessage();
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
