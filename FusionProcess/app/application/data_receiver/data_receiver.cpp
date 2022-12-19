#include "data_receiver.h"
#include <functional>

namespace
{
    const std::string tcp_connect_addr = "192.168.198.131";
    const std::string tcp_connect_port = "35243";
    const short tcp_listen_port = 56842;
    const short udp_camera_port = 51240;
    const short udp_radar_port = 52240;
}

DataReceiver::DataReceiver(asio::io_context& ioService)
             : client(ioService, tcp_connect_addr, tcp_connect_port,
                      std::bind(&DataReceiver::ProcessReadMsg, this, std::placeholders::_1)),
               server(ioService, tcp_listen_port,
                      std::bind(&DataReceiver::ProcessReadMsg, this, std::placeholders::_1)),
               prescan_camera_server(ioService, udp_camera_port,
                      std::bind(&DataReceiver::ProcessPrescanCamera, this, std::placeholders::_1, std::placeholders::_2)),
               prescan_radar_server(ioService, udp_radar_port,
                      std::bind(&DataReceiver::ProcessPrescanRadar, this, std::placeholders::_1, std::placeholders::_2))
{
}

DataReceiver::~DataReceiver()
{
    client.close();
}

void DataReceiver::Start()
{
    client.start();
}

void DataReceiver::ProcessReadMsg(FusionChatMessage msg)
{
    process.ProcessReadMsg(std::move(msg));
}

void DataReceiver::ProcessPrescanCamera(char* data, uint32_t len)
{
    prescan_process.PrescanReadCameraMsg(data);
}

void DataReceiver::ProcessPrescanRadar(char* data, uint32_t len)
{
    prescan_process.PrescanReadRadarMsg(data);
}

