
#include <sys/time.h>
#include <iostream>

#include "base64.hpp"
#include "cds_proxy.h"
#include "ipc_data.h"
#include "cdsproxy_tcp_message.h"
#include "cdsproxy_cds_header.h"

using nlohmann::json;


const std::map<MsgType, size_t> CdsProxy_MessageDefinition = {
    {MsgType::IPC_CAN,  sizeof(IPC_CAN_Data)},
};

CdsProxy::CdsProxy(asio::io_context& ioService, short port) : IProxy(ioService, MsgType::CDS), server(ioService, MsgType::CDS, port)
{

}

void CdsProxy::Init()
{
    server.start();
}

void CdsProxy::ProcessIncomingMessage(MsgType msgType, uint8_t* data, uint16_t len)
{
    //  std::cout << "CdsProxy::ProcessIncomingMessage"  << std::endl;

    auto iter = CdsProxy_MessageDefinition.find(msgType);
	if (iter == CdsProxy_MessageDefinition.end())
	{
		//  std::cout << "undefined message type " << int(msgType) << std::endl;
		return;
	}
	if (iter->second != len)
	{
		//  std::cout << "mismatched message length" << int(msgType) << std::endl;
		return;
	}
	switch (msgType)
	{
	case MsgType::IPC_CAN: {
			cds_CanData = *((IPC_CAN_Data *)(data));
			// std::cout << "antiLockBrakeStatus  " << cds_CanData.antiLockBrakeStatus << std::endl;
		}
		break;
	default:
		break;
	} 
}

void CdsProxy::SendperiodMessage()
{
    std::string candata = base64_encode((const unsigned char*)&cds_CanData, sizeof(IPC_CAN_Data));
    json data;
    data["Data"] = candata.c_str();
    // std::cout << " json data['Data'] = " << data.dump() << std::endl;
    addMsgHeadAndSend(data.dump().c_str(), data.dump().size());
    
}

void CdsProxy::addMsgHeadAndSend(const char* data, unsigned int len)
{
    MsgHead_t head;
    memset(&head, 0x00, sizeof(head));
        
    head.head0 = 0x00;
    head.head1 = 0xff;
        
    head.msgtype = 0x01;
    head.format = 0x01;
    head.timestamp = getSytemTime();
        
    head.msgid = 1;       
        
    head.datalen = len;
        
    char sendbuf[1024];
    memset(sendbuf, 0, sizeof(sendbuf));
    memcpy(sendbuf , (char*)&head, sizeof(MsgHead_t) );
    memcpy(sendbuf + sizeof(MsgHead_t), data, len);

    
    DoServerWrite(sendbuf, sizeof(MsgHead_t) + len);
    // std::cout<<"addMsgHeadAndSend over,buf len="<<(sizeof(MsgHead_t))<<"+"<<len<<"="<<(sizeof(MsgHead_t) + len)<<std::endl;      
       
 }

void CdsProxy::DoServerWrite(const char* buf , const uint16_t len)
{ 
    CdsTcpMessage msg(buf, len); 
    server.write(msg);
}

uint64_t CdsProxy::getSytemTime()
{
    struct timeval tv;
    if (gettimeofday(&tv, nullptr) < 0)
    {
        return -1;;
    }
        
    return ((uint64_t)tv.tv_sec) * 1000 * 1000 + tv.tv_usec;
}