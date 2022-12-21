
#include <cstdlib>
#include <iostream>
#include "asio.hpp"
#include "hmi_pad_data.h"
#include "nlohmann/json.hpp"
#include <stdio.h>      
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <string.h> 
#include <arpa/inet.h>
#include <glog/logging.h>

std::string getLocalIpAddress()
{
    struct ifaddrs * ifAddrStruct=NULL;
    struct ifaddrs * ifa=NULL;
    void * tmpAddrPtr=NULL;

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (!ifa->ifa_addr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET)
        { // check it is IP4, is a valid IP4 Address
            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            std::string ipAddr(addressBuffer, strlen(addressBuffer));
            if (ipAddr.compare(0, 9, "192.168.20") == 0)
            {
                if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
                return ipAddr;
            }
            LOG(INFO) << ifa->ifa_name << "get Host IP Address: " << addressBuffer; 
        }
    }
    if (ifAddrStruct != NULL)
    {
        LOG(INFO) << "get Host IP Address error, return default ip";
        freeifaddrs(ifAddrStruct);
    }
    return "192.168.20.11";
}

using json = nlohmann::json;
using asio::ip::udp;

class HmiUdpAddressingServer
{
public:
    static constexpr int AddressReqPort = 50500;
    static constexpr int AddressRespPort = 50501;

public:
    HmiUdpAddressingServer(asio::io_context& io_context, int hmi_tcp_port)
        : socket_(io_context, udp::endpoint(asio::ip::address_v4::any()
        , AddressReqPort))
        , tcp_port(hmi_tcp_port)
    {
    }

    void start()
    {
        LOG(INFO) << "udp addressing server is started";
        do_receive();
    }

    void do_receive()
    {
        socket_.async_receive_from(
            asio::buffer(data_, max_length), sender_endpoint_,
                [this](std::error_code ec, std::size_t bytes_recvd)
          {
              if (!ec && bytes_recvd > 0)
              {
                  if (!json::accept(data_))
                  {
                      LOG(INFO) << "Invalid input, parse error ";
                     return ;
                  }
                  json j = json::parse(data_,data_ + bytes_recvd);
                  int tag = j["tag"];
                  PadAddressingResponseFrame frame;
                  LOG(INFO) << "udp received tag: " << tag;
                  if (tag == 1)
                  {
                      std::string destinationIp = j["destinationIp"];
                      LOG(INFO) << "udp received destinationIp: " << destinationIp;
                      frame.tag = 0x0002;
                      frame.rsp = 0;
                      frame.detail = "return host ipaddr";
                      frame.data.gSentryIp = getLocalIpAddress();
                      frame.data.gSentryPort = tcp_port;
                      frame.data.deviceSerialNum = "CDC1X32";
                  }
                  else
                  {
                      frame.tag = 0x0002;
                      frame.rsp = 1;
                      frame.detail = "return host ipaddr failure!";
                      frame.data.gSentryIp = "";
                      frame.data.gSentryPort = -1;
                      frame.data.deviceSerialNum = "-1";
                  }
              do_send(frame);
          }
          else
          {
          }
        });
    }

    void do_send(PadAddressingResponseFrame& Frame)
    {
        json out = json(Frame);
        std::string str = out.dump();
        LOG(INFO) << "udp response: " << str;
        auto endpoint = udp::endpoint(sender_endpoint_.address(), AddressRespPort);

        socket_.async_send_to(
            asio::buffer(str.c_str(), str.length()), endpoint,
                [this](std::error_code /*ec*/, std::size_t /*bytes_sent*/)
            {
                do_receive();
            });
    }
  // void start()
  // {
  //   socket_.async_receive_from(
  //       asio::buffer(data_, max_length), sender_endpoint_,
  //       [this](std::error_code ec, std::size_t bytes_recvd)
  //       {
  //         if (!ec && bytes_recvd > 0)
  //         {
  //           LOG(INFO) << "responce";
  //           json j = json::parse(data_, data_ + bytes_recvd);;
  //           int tag = j["tag"];
  //           LOG(INFO) << "udp received tag: " << tag;
  //           if (tag == 1) {
  //             std::string destinationIp = j["destinationIp"];
  //             LOG(INFO) << "udp received destinationIp: " << destinationIp;
  //             PadAddressingResponseFrame frame;
  //             frame.tag = 0x0002;
  //             frame.rsp = 0;
  //             frame.detail = "";
  //             frame.data.gSentryIp = getLocalIpAddress();
  //             // frame.data.gSentryIp = "172.16.0.88",
  //             frame.data.gSentryPort = tcp_port;
  //             frame.data.deviceSerialNum = "CDC1X32";
  //             json out = json(frame);
  //             std::string str = out.dump();
  //             LOG(INFO) << "udp response: " << str;

  //             auto endpoint = udp::endpoint(sender_endpoint_.address(), AddressRespPort);

  //             socket_.async_send_to(
  //                 asio::buffer(str.c_str(), str.length()), endpoint,
  //                 [this](std::error_code /*ec*/, std::size_t /*bytes_sent*/)
  //                 {
  //                 });
  //           }
  //         }
  //         else
  //         {
  //             LOG(INFO) << "receive error!";
  //         }
  //       });
  // }

private:
  udp::socket socket_;
  udp::endpoint sender_endpoint_;
  enum { max_length = 1024 };
  char data_[max_length];
  int tcp_port;
};