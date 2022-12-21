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

CdsProxy::CdsProxy(asio::io_context& ioService, short port) : IProxy(ioService, MsgType::CDS), m_Server(ioService, MsgType::CDS, port)
{
    memset(&m_Cds_CanData, 0, sizeof(m_Cds_CanData));
    m_Cds_CanData.brakePedalStatus = 1;
    m_Cds_CanData.brakeAppliedStatus = 2;
}

void CdsProxy::ProcessIncomingMessage(MsgType msgType, uint8_t* data, uint16_t len)
{
    auto iter = CdsProxy_MessageDefinition.find(msgType);
	if (iter == CdsProxy_MessageDefinition.end())
	{
		return;
	}
	if (iter->second != len)
	{
		return;
	}
	switch (msgType)
	{
	case MsgType::IPC_CAN:
    {
		m_Cds_CanData = *((IPC_CAN_Data *)(data));
	}
		break;
	default:
		break;
	} 
}

void CdsProxy::SendperiodMessage_50ms()
{
    IPC_CAN_Data can_data;
    can_data.transmissionState = htonl(m_Cds_CanData.transmissionState);
    can_data.steeringWheelAngle = htonl(m_Cds_CanData.steeringWheelAngle);
    can_data.brakePedalStatus = htonl(m_Cds_CanData.brakePedalStatus);
    can_data.brakeAppliedStatus = htonl(m_Cds_CanData.brakeAppliedStatus);
    can_data.tractionControlStatus = htonl(m_Cds_CanData.tractionControlStatus);
    can_data.antiLockBrakeStatus = htonl(m_Cds_CanData.antiLockBrakeStatus );
    can_data.stabilityControlStatus = htonl(m_Cds_CanData.stabilityControlStatus);
    can_data.brakeBoostApplied = htonl(m_Cds_CanData.brakeBoostApplied);
    can_data.auxiliaryBrakesStatus = htonl(m_Cds_CanData.auxiliaryBrakesStatus);
    can_data.eventHazardLights = htonl(m_Cds_CanData.eventHazardLights);
    can_data.eventABSactivated = htonl(m_Cds_CanData.eventABSactivated);
    can_data.eventTractionControlLoss = htonl(m_Cds_CanData.eventTractionControlLoss);
    can_data.eventStabilityControlactivated = htonl(m_Cds_CanData.eventStabilityControlactivated);
    can_data.eventHardBraking = htonl(m_Cds_CanData.eventHardBraking);
    can_data.eventFlatTire = htonl(m_Cds_CanData.eventFlatTire);
    can_data.eventDisabledVehicle = htonl(m_Cds_CanData.eventDisabledVehicle);
    can_data.eventAirBagDeployment = htonl(m_Cds_CanData.eventAirBagDeployment);
    can_data.eventWipersStatus = htonl(m_Cds_CanData.eventWipersStatus);
    can_data.exteriorLights = htonl(m_Cds_CanData.exteriorLights);
    can_data.vehicleSpeed = htonl(m_Cds_CanData.vehicleSpeed);
    can_data.enduranceMileage = htonl(m_Cds_CanData.enduranceMileage);
    std::string candatabuf = base64_encode((const unsigned char*)&can_data, sizeof(IPC_CAN_Data));
    if(!candatabuf.empty())
    {
        json data;
        data["Data"] = candatabuf.c_str();
        LOG(INFO) << " json data['Data'] = " << data.dump();
        addMsgHeadAndSend(data.dump().c_str(), data.dump().size());
    }
    else
    {
        LOG(INFO) << " candata after base64encode is null";
    }
}

void CdsProxy::Init()
{
    m_Server.start();
}

void CdsProxy::addMsgHeadAndSend(const char* data, unsigned int len)
{
    MsgHead_t head;
    memset(&head, 0x00, sizeof(head));
    head.head0 = 0x00;
    head.head1 = 0xff;

    // 预留
    // head.msgtype = 0x01;
    // head.format = 0x01;
    // head.timestamp = getSytemTime();
    // head.msgid = 1;

    head.datalen = htonl(len);
    char sendbuf[1024];
    memset(sendbuf, 0, sizeof(sendbuf));
    memcpy(sendbuf , (char*)&head, sizeof(MsgHead_t) );
    memcpy(sendbuf + sizeof(MsgHead_t), data, len);

    DoServerWrite(sendbuf, sizeof(MsgHead_t) + len);
 }

void CdsProxy::DoServerWrite(const char* buf , const uint16_t len)
{
    m_Server.write(CdsTcpMessage(buf, len));
}