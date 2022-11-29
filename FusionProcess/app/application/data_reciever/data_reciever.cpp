#include "data_reciever.h"
#include <functional>

DataReciever::DataReciever(asio::io_context& ioService, std::string ipAddr, std::string port)
             : client(ioService, ipAddr, port)
{
    client.set_read_cb(std::bind(&DataReciever::ProcessReadMsg, this, std::placeholders::_1));
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
