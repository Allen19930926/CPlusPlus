#include "data_reciever.h"
#include <functional>

DataReciever::DataReciever(asio::io_context& ioService, std::string ipAddr, std::string port, short listen_port)
             : client(ioService, ipAddr, port, std::bind(&DataReciever::ProcessReadMsg, this, std::placeholders::_1)),
               server(ioService, listen_port, std::bind(&DataReciever::ProcessReadMsg, this, std::placeholders::_1))
{
}

DataReciever::~DataReciever()
{
    client.close();
}

void DataReciever::Start()
{
    client.start();
}

void DataReciever::ProcessReadMsg(FusionChatMessage msg)
{
    process.ProcessReadMsg(std::move(msg));
}
