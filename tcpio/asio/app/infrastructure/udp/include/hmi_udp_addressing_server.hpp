
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

std::string getLocalIpAddress() {
    struct ifaddrs * ifAddrStruct=NULL;
    struct ifaddrs * ifa=NULL;
    void * tmpAddrPtr=NULL;

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            std::string ipAddr(addressBuffer, strlen(addressBuffer));
            if (ipAddr.compare(0, 9, "192.168.20") == 0) {
              if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
              return ipAddr;
            }
            // printf("%s IP Address %s\n", ifa->ifa_name, addressBuffer); 
        }
    }
    if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);
    return "192.168.20.153";
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
    : socket_(io_context, udp::endpoint(asio::ip::address_v4::any(), AddressReqPort)), tcp_port(hmi_tcp_port)
  {
    std::cout << "udp addressing server is started" << std::endl;

    socket_.set_option(udp::socket::broadcast(true));
    // asio::ip::address multicast_address = asio::ip::address::from_string("255.255.255.255");
    // asio::ip::multicast::join_group option(multicast_address);
    // socket_.set_option(option);
  }

  void start()
  {
    socket_.async_receive_from(
        asio::buffer(data_, max_length), sender_endpoint_,
        [this](std::error_code ec, std::size_t bytes_recvd)
        {
          if (!ec && bytes_recvd > 0)
          {
            json j = json::parse(data_, data_ + bytes_recvd);;
            int tag = j["tag"];
            std::cout << "udp received tag: " << tag <<std::endl;
            if (tag == 1) {
              std::string destinationIp = j["destinationIp"];
              std::cout << "udp received destinationIp: " << destinationIp <<std::endl;
              PadAddressingResponseFrame frame;
              frame.tag = 0x0002;
              frame.rsp = 0;
              frame.detail = "";
              frame.data.gSentryIp = getLocalIpAddress();
              frame.data.gSentryPort = tcp_port;
              frame.data.deviceSerialNum = "CDC1X32";
              json out = json(frame);
              std::string str = out.dump();
              std::cout << "udp response: " << str <<std::endl;

              auto endpoint = udp::endpoint(sender_endpoint_.address(), AddressRespPort);

              socket_.async_send_to(
                  asio::buffer(str.c_str(), str.length()), endpoint,
                  [this](std::error_code /*ec*/, std::size_t /*bytes_sent*/)
                  {
                  });
            }
          }
          else
          {
          }
        });
  }

private:
  udp::socket socket_;
  udp::endpoint sender_endpoint_;
  enum { max_length = 1024 };
  char data_[max_length];
  int tcp_port;
};