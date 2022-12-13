#include <cstdint>
#include <math.h>
#include <cstring>
#include <glog/logging.h>
#include <sstream>
#include <string>
#include <set>

#include "asio/steady_timer.hpp"
#include "hmi_proxy.h"
#include "adasMath.h"
#include "event_msg.h"
#include "event_queue.h"
#include "ipc_data.h"
#include "hmi_gSentry.h"
#include "hmi_pad_data.h"
#include "hmi_message.h"
#include "hmi_tcp_message.h"

#include "data_base.h"

#define HMI_REQUEST_TIMEOUT 100

#define TIMEOUT -1  // 对应rsp回复失败（！0）
#define RESP_MISMATCH -2  // 对应rsp回复失败（！0）
#define GENERAL_FAIL -3  // 对应rsp回复失败（！0）
#define SERVER_BUSY -4  // 对应rsp回复失败（！0）

const std::map<int, std::string> HmiProxy_CACCDecisionDefinition = {
	{1, "不在路口"},
	{2, "红灯停车"},
	{3, "跟停前车"}
};

const std::map<MsgType, size_t> HmiProxy_MessageDefinition = {
	{MsgType::IPC_GNSS_DATA, sizeof(IPC_GNSS_Data)},
	{MsgType::IPC_SYS_ERROR, sizeof(IPC_System_Error)},
	{MsgType::IPC_EVH, sizeof(EVH_SubjectInfo_BUS)},
	{MsgType::IPC_HMI_INFO, sizeof(FuncCoord_FAM_HMI_Info)},
};

const std::set<HMI_PAD_TAG_DEF> HmiProxy_CtrTagFromPAD = {
	HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_ON_REQ,
	HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_OFF_REQ,
	HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_RESET_REQ,
	HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_CC_REQ,
	HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_TIME_REQ,
	HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_ON_REQ,
	HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_OFF_REQ,
	HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_MODE_REQ,
	HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_SENS_REQ,
	HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_ON_REQ,
	HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_OFF_REQ,
	HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_AEB_ON_REQ,
	HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_AEB_OFF_REQ
};

HmiProxy::HmiProxy(asio::io_context& ioService, std::string clientIp, std::string clientPort, short listenPort)
    : IProxy(ioService, MsgType::HMI_DUMMY)
	, m_Server(ioService, MsgType::HMI_PAD, listenPort)
	, m_Client(ioService, MsgType::HMI_GSENTRY, clientIp, clientPort)
	, m_TimeoutTimer(ioService)
{
}

void HmiProxy::ProcessRecieveIPCData(MsgType msgType, uint8_t* data, uint16_t len)
{
	LOG(INFO) << "msg from IPC: " << static_cast<int>(msgType);
	auto iter = HmiProxy_MessageDefinition.find(msgType);
	if (iter == HmiProxy_MessageDefinition.end())
	{
		LOG(INFO) << "undefined message type " << int(msgType);
		return;
	}
	if (iter->second != len)
	{
		LOG(INFO) << "mismatched message length, length is " << len;
		return;
	}
	switch (msgType)
	{
	case MsgType::IPC_GNSS_DATA:
	{
		IPC_GNSS_Data* gnssData = (IPC_GNSS_Data *)(data);
		LOG(INFO) << "IPC_GNSS_DATA from IPC: " 
			<< "\n\theadingAngle: " << gnssData->headingAngle
			<< "\n\tlatitude: " << gnssData->latitude
			<< "\n\tlongitude: " << gnssData->longitude;
		m_HostVehInfo.data.heading = round(gnssData->headingAngle * 100) / 100;
		LOG(INFO) << " m_HostVehInfo.data.heading " << m_HostVehInfo.data.heading;
		m_HostVehInfo.data.latitude = gnssData->latitude;
		m_HostVehInfo.data.longitude = gnssData->longitude;
	}; break;
	case MsgType::IPC_SYS_ERROR:
	{
		std::vector<PadSystemErrorFrame>().swap(m_SysErrorVec.data.systemError);
		m_IPC_systemError = *(IPC_System_Error *)data;
		for (int i = 0; i < IPC_MAX_SYSTEM_ERROR_NUM; i++) {
			if (m_IPC_systemError.De_Fim_Fids_u8[i] != 0) {
				PadSystemErrorFrame p;
				p.type = m_IPC_systemError.De_Fim_Fids_u8[i];
				p.detail = "";
				m_SysErrorVec.data.systemError.push_back(p);
			}
		}
	}; break;

	case MsgType::IPC_EVH:
	{
		EVH_SubjectInfo_BUS* hostInfo = (EVH_SubjectInfo_BUS*)(data);
		LOG(INFO) << "IPC_EVH from IPC: " 
			<< "\n\tDe_ego_vxMs_f32: " << hostInfo->De_ego_vxMs_f32
			<< "\n\tDe_ego_axMs2_f32: " << hostInfo->De_ego_axMs2_f32
			<< "\n\tDe_ego_axMs2_f32: " << hostInfo->De_ego_axMs2_f32;
		m_HostVehInfo.data.speed = round(ADASMath::convertSpeedM2Km(round(hostInfo->De_ego_vxMs_f32)));
		m_HostVehInfo.data.acceleration = round(sqrtf(powf(hostInfo->De_ego_axMs2_f32, 2)
		 + powf(hostInfo->De_ego_ayMs2_f32, 2)) * 10) / 10;
	}; break;

	case MsgType::IPC_HMI_INFO: 
	{
		FuncCoord_FAM_HMI_Info* tmp_getMcoreInfo = (FuncCoord_FAM_HMI_Info *)(data);
		if(nullptr == data)
			LOG(INFO) << "IPC_HMI_INFO: receive IPC data ia null";
        LOG(INFO) << "\n\tIPC_HMI_INFO from IPC: "
              << "\n\ttmp_getMcoreInfo->De_ResponseCounter_u32:" << (int)tmp_getMcoreInfo->De_ResponseCounter_u32
              << "\n\ttmp_getMcoreInfo->De_ADAS_FCW_u8:" << (int)tmp_getMcoreInfo->De_ADAS_FCW_u8
              << "\n\ttmp_getMcoreInfo->De_AEB_Response_u8:" << (int)tmp_getMcoreInfo->De_AEB_Response_u8
              << "\n\ttmp_getMcoreInfo->De_AEB_Triger_u8:" << (int)tmp_getMcoreInfo->De_AEB_Triger_u8
              << "\n\ttmp_getMcoreInfo->De_CACC_ACCStatus_u8:" << (int)tmp_getMcoreInfo->De_CACC_ACCStatus_u8
              << "\n\ttmp_getMcoreInfo->De_CACC_CACCOff_Rsp_u8:" << (int)tmp_getMcoreInfo->De_CACC_CACCOff_Rsp_u8
              << "\n\ttmp_getMcoreInfo->De_CACC_CACCOn_Rsp_u8:" << (int)tmp_getMcoreInfo->De_CACC_CACCOn_Rsp_u8
              << "\n\ttmp_getMcoreInfo->De_CACC_CACCResume_Rsp_u8:" << (int)tmp_getMcoreInfo->De_CACC_CACCResume_Rsp_u8
              << "\n\ttmp_getMcoreInfo->De_CACC_CACCStatus_u8:" << (int)tmp_getMcoreInfo->De_CACC_CACCStatus_u8
              << "\n\ttmp_getMcoreInfo->De_CACC_Decision_u8:" << (int)tmp_getMcoreInfo->De_CACC_Decision_u8
              << "\n\ttmp_getMcoreInfo->De_CACC_IDAOff_Rsp_u8:" << (int)tmp_getMcoreInfo->De_CACC_IDAOff_Rsp_u8
              << "\n\ttmp_getMcoreInfo->De_CACC_IDAOn_Rsp_u8:" << (int)tmp_getMcoreInfo->De_CACC_IDAOn_Rsp_u8
              << "\n\ttmp_getMcoreInfo->De_CACC_IDAStatus_u8:" << (int)tmp_getMcoreInfo->De_CACC_IDAStatus_u8
              << "\n\ttmp_getMcoreInfo->De_CACC_SpeedSet_Rsp_u8:" << (int)tmp_getMcoreInfo->De_CACC_SpeedSet_Rsp_u8
              << "\n\ttmp_getMcoreInfo->De_CACC_SportMode_Rsp_u8:" << (int)tmp_getMcoreInfo->De_CACC_SportMode_Rsp_u8
              << "\n\ttmp_getMcoreInfo->De_CACC_TimeGapSet_Rsp_u8:" << (int)tmp_getMcoreInfo->De_CACC_TimeGapSet_Rsp_u8
              << "\n\ttmp_getMcoreInfo->De_FCW_AEB_FuncSts_u8:" << (int)tmp_getMcoreInfo->De_FCW_AEB_FuncSts_u8
              << "\n\ttmp_getMcoreInfo->De_gSentry_Warning_u8:" << (int)tmp_getMcoreInfo->De_gSentry_Warning_u8;

		if(tmp_getMcoreInfo->De_CACC_CACCStatus_u8 != m_FromMcoreInfo.De_CACC_CACCStatus_u8 || 
		   tmp_getMcoreInfo->De_CACC_IDAStatus_u8 != m_FromMcoreInfo.De_CACC_IDAStatus_u8)
		{
			m_CACCStatus.data.ACCStatus = tmp_getMcoreInfo->De_CACC_CACCStatus_u8;
			if(3 == m_CACCStatus.data.ACCStatus)
			{
				m_CACCStatus.data.ACCfaultDetail = 3;
			}
			m_CACCStatus.data.IDAStatus = tmp_getMcoreInfo->De_CACC_IDAStatus_u8; 

			if(3 == m_CACCStatus.data.IDAStatus)
			{
				m_CACCStatus.data.IDAfaultDetail = 3;
			}
			m_CACCStatus.data.reason = "reserve";

			SendCACCStatus();
		}

		if(tmp_getMcoreInfo->De_FCW_AEB_FuncSts_u8 != m_FromMcoreInfo.De_FCW_AEB_FuncSts_u8)
		{
			m_CAEBStatus.data.CAEBStatus = tmp_getMcoreInfo->De_FCW_AEB_FuncSts_u8;
			if(3 == m_CAEBStatus.data.CAEBStatus)
			{
				m_CAEBStatus.data.faultDetail = "Fault happen!!!";
			}
			SendCAEBStatus();
		}

		if (tmp_getMcoreInfo->De_CACC_Decision_u8 != m_FromMcoreInfo.De_CACC_Decision_u8) {
			if (m_FromMcoreInfo.De_CACC_Decision_u8 >= 20) {
				std::stringstream fmt;
				fmt << "绿波通行，车速为" << m_FromMcoreInfo.De_CACC_Decision_u8 << "KPH";
				m_CACCDecision.data.CACCDecision =  fmt.str();
			}
			else if (m_FromMcoreInfo.De_CACC_Decision_u8 > 0 && m_FromMcoreInfo.De_CACC_Decision_u8 <= 3)
			{
				auto iter = HmiProxy_CACCDecisionDefinition.find(m_FromMcoreInfo.De_CACC_Decision_u8);
				m_CACCDecision.data.CACCDecision =  iter->second;
			}
			SendCACCDecisionInfo();
		}

		m_CAEBDecision.data.preWarning = tmp_getMcoreInfo->De_ADAS_FCW_u8;
		m_CAEBDecision.data.CAEB = tmp_getMcoreInfo->De_AEB_Triger_u8;

		m_FromMcoreInfo = *tmp_getMcoreInfo;

		//receive IPC_HMI_INFO msg, response to PAD
		HandleRecieveIPCRespMsg();
	}
		break;

	default:
		LOG(INFO) << "no excepted msgTYpe from IPC";
		break;
	}
}

void HmiProxy::ProcessRecieveGsentryData(MsgType msgType, uint8_t* data, uint16_t len)
{
	if(msgType == MsgType::HMI_GSENTRY)
	{
		LOG(INFO) << "msg from gSentry(HMI): " << static_cast<int>(msgType);
		HandlegSentryMessage(data, len);
	}
	else
	{
		LOG(INFO) << "Not SentryMessage! ";
	}
}

void HmiProxy::ProcessRecievePadData(MsgType msgType, uint8_t* data, uint16_t len)
{
	if(msgType == MsgType::HMI_PAD)
	{
		LOG(INFO) << "msg from PAD: " << static_cast<int>(msgType);
		HandlePadMessage(data, len);
	}
	else
	{
		LOG(INFO) << "Not PadMessage! ";
	}
}

void HmiProxy::DoPeriodTask_200ms()
{
	SendHostVeh_CIPVInfo();
	SendV2XObjVehInfo();
}

void HmiProxy::DoPeriodTask_50ms()
{
	SendSysErrorStatus();
	SendCAEBDecisionInfo(); 
}

void HmiProxy::Init()
{
    m_Server.start();
    m_Client.start();
    KeepAlive(1000);
}

void HmiProxy::KeepAlive(const uint32_t interval)
{
    SetPeriodTask(interval, std::bind(&HmiProxy::SendClientAliveMsg, this));
}

void HmiProxy::SendClientAliveMsg()
{
    gSentryKeepAliveRequestFrame frame;
    frame.tag = static_cast<int>(HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_KEEP_ALIVE_REQ);
    json j;
    to_json(j, frame);
    m_Client.write(j.dump());
}

void HmiProxy::DoClientWrite(HmiTcpMessage& msg)
{
	LOG(INFO) << "client send to gSentry(HMI) Body: " << msg.Body() << " BodyLength: " << msg.BodyLength();
    m_Client.write(msg);
}

void HmiProxy::DoServerWrite(HmiTcpMessage& msg)
{
	LOG(INFO) << "server send to PAD Body: " << msg.Body() << " BodyLength: " << msg.BodyLength();
    m_Server.write(msg);
}

template <typename T>
void HmiProxy::DoServerWrite(T& msg)
{
	HmiTcpMessage tcpMsg = HmiTcpMessage(HmiMessage(msg));
    LOG(INFO) << "server send to PAD Body(T): " << tcpMsg.Body() << " BodyLength: " << tcpMsg.BodyLength();
    m_Server.write(tcpMsg);
}

void HmiProxy::HandlegSentryMessage(uint8_t* data, uint16_t len)
{
    if (!json::accept(data))
    {
        LOG(INFO) << "Invalid input, parse error ";
        return ;
    }

    json j= json::parse(data);
    if (!j.contains("tag"))
    {
        LOG(INFO) << "Invalid input, no tag subject! ";
        return ;
    }

    if (!j["tag"].is_number())
    {
		LOG(INFO) << "tag is not number! ";
        return ;
    }

	HMI_GSENTRY_TAG_DEF tag = HMI_GSENTRY_TAG_DEF(j["tag"]);
	if (HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_HOSTDATA == tag ||
  		HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_REMOTEDATA == tag ||
  		HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_GLOSAINFO == tag)
	{
		return; /// filter tag 1007 1008 1009
	}

	LOG(INFO) << "client receive from gSentry(HMI) data: " << data << " len: " << len;

	switch(tag)
	{
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_INIT_RESP:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_CLOSEV2X_RESP:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_OPENV2X_RESP:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_CLOSEONEV2X_RESP:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_OPENONEV2X_RESP:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_SENCE_RESP:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_GSENTRY_STATUS_REQ:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_V2X_STATUS_RESP:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_V2XSWITCH_STATUS_RESP:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_SENSESTATUS_RESP:
		{
			auto hmiMsg = HmiTcpMessage(HmiMessage(data, len));
			DoServerWrite(hmiMsg);
		}
		break;

		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_CAEB_COLLISION_SENS_REP:
		{
			 /* Process Adas caution attention */
        	ProcessCAEBCollisionWarning(j);
			auto hmiMsg = HmiTcpMessage(HmiMessage(data, len));
			DoServerWrite(hmiMsg);
		}
		break;

		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_V2IWARNING:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_RLVWINFO:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_VRUCWINFO:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_TRAFFIClIGHT:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_SLWINFO:
		{
			auto hmiMsg = HmiTcpMessage(HmiMessage(data, len));
			DoServerWrite(hmiMsg);
		}
		break;

		default:
		LOG(INFO) << "unexpected tag from gSentry(HMI):" << static_cast<int>(tag);
		break;
	}
}

void HmiProxy::ProcessCAEBCollisionWarning(json& j)
{
    const uint32_t FCW_WARN = 1; // filter FCW from gSentry(HMI)
    if (!j.contains("data"))
    {
        LOG(INFO) << "Invalid input, no data subject! ";
        return ;
    }

    json& dataNode = j["data"];
    if (!dataNode.contains("warningInfo"))
    {
        LOG(INFO) << "Invalid input, no warningInfo subject! ";
        return ;
    }

    json& warnNode = dataNode["warningInfo"];
    if (!warnNode.is_array())
    {
        LOG(INFO) << "Invalid input, warningInfo is not array! ";
        return ;
    }

    uint32_t fcwIndex = 0;
    bool     fcwExist = false;
    for (uint32_t i=0; i<warnNode.size(); i++)
    {
        if (!warnNode[i].contains("warningType"))
        {
            continue;
        }
        if (warnNode[i]["warningType"] == FCW_WARN)
        {
            //替换成IPCM传递的预警信息
            fcwExist = m_FromMcoreInfo.De_gSentry_Warning_u8 == 1 ? false : true;
            fcwIndex = i;
        }
    }

    if (fcwExist)
    {
        warnNode.erase(fcwIndex);
    }
}

void HmiProxy::HandlePadMessage(uint8_t* data, uint16_t len) {
    
	HmiMessage msg(data, len);
	HMI_PAD_TAG_DEF tag = HMI_PAD_TAG_DEF(msg.GetTag());
	LOG(INFO) << " server receive data from pad: " << data <<"  len: " << len;
	// PAD连接请求响应(连接Hmiproxy,而不是gentry -> Hmiproxy进行响应)
	if (HMI_PAD_TAG_DEF::HMI_TAG_PAD_CON_REQ == tag)
	{
		PadConnectResponseFrame frame;  
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CON_RESP);
		frame.rsp = 0;
		frame.data.deviceSerialNum = "CDC1X32";
		DoServerWrite(frame);

		// ADAS系统开关状态上报，系统每次开机需主动上报 目前default value
		m_SwitchSetingInfo.data.CACCSwitch = 0;
		m_SwitchSetingInfo.data.IDASwitch = 0; 
		m_SwitchSetingInfo.data.currentTimeHeadway = 0;
		m_SwitchSetingInfo.data.CAEBWarningSwitch = 0;
		m_SwitchSetingInfo.data.currentCollisionSensitivity = 0;
		m_SwitchSetingInfo.data.CAEBSwitch = 0;
		SendSwitchSettings(); 
		return;
	}

	// 心跳探测(连接Hmiproxy,而不是gentry -> Hmiproxy进行响应)
	if (HMI_PAD_TAG_DEF::HMI_TAG_PAD_KEEP_ALIVE_REQ == tag)
	{
		LOG(INFO) << "keep alive request is received";
		PadKeepAliveResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_KEEP_ALIVE_RESP);
		frame.rsp = 0;
		frame.data.deviceNum = "CDC1X32";
		DoServerWrite(frame);
		return;
	}

	// 处理PAD对gsentry(hmi)控制类请求
	switch(tag)
	{
		case HMI_PAD_TAG_DEF::HMI_TAG_PAD_INIT_REQ:
		case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CLOSEV2X_REQ:
		case HMI_PAD_TAG_DEF::HMI_TAG_PAD_OPENV2X_REQ:
		case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CLOSEONEV2X_REQ:
		case HMI_PAD_TAG_DEF::HMI_TAG_PAD_OPENONEV2X_REQ:
		case HMI_PAD_TAG_DEF::HMI_TAG_PAD_SENSE_REQ:
		case HMI_PAD_TAG_DEF::HMI_TAG_PAD_GSENTRY_STATUS_RESP:
		case HMI_PAD_TAG_DEF::HMI_TAG_PAD_V2X_STATUS_REQ:
		case HMI_PAD_TAG_DEF::HMI_TAG_PAD_V2XSWITCH_STATUS_REQ:
		case HMI_PAD_TAG_DEF::HMI_TAG_PAD_SENSESTATUS_REQ:
		{
			// 将请求转给gsentry(hmi)
			auto HmiMsg = HmiTcpMessage(msg);
			DoClientWrite(HmiMsg);
			return;
		}

		default:
			LOG(INFO) << "##not gsentry(hmi) control msg request, continue to handle the request for Mcore!!!";
			break;
	}

	/********************处理PAD对M核的控制类功能请求（带超时处理机制）*******************/
	if(HmiProxy_CtrTagFromPAD.end() == HmiProxy_CtrTagFromPAD.find(tag))
	{
		LOG(INFO) << "the tag from PAD is invalid , return!!!";
		return;
	}
	m_PendingRequest = tag;  // 暂存PAD对M核 的控制类请求tag
	m_Counter++;
	switch (tag) 
	{
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_ON_REQ:
	{
		m_ToMcoreInfo.De_RequestCounter_u32 = m_Counter;
		m_ToMcoreInfo.De_CACC_Switch_u8 = true;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&m_ToMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		LOG(INFO) << "m_ToMcoreInfo.De_RequestCounter_u32: " << m_ToMcoreInfo.De_RequestCounter_u32 
			<< "  m_ToMcoreInfo.De_CACC_Switch_u8" << (int)m_ToMcoreInfo.De_CACC_Switch_u8;
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_OFF_REQ: {
		m_ToMcoreInfo.De_RequestCounter_u32 = m_Counter;
		m_ToMcoreInfo.De_CACC_Switch_u8 = false;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&m_ToMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		LOG(INFO) << "m_ToMcoreInfo.De_RequestCounter_u32: " << m_ToMcoreInfo.De_RequestCounter_u32 
			<< "  m_ToMcoreInfo.De_CACC_Switch_u8" << (int)m_ToMcoreInfo.De_CACC_Switch_u8;
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_RESET_REQ: {
		m_ToMcoreInfo.De_RequestCounter_u32 = m_Counter;
		m_ToMcoreInfo.De_CACC_Resume_u8 = true;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&m_ToMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		LOG(INFO) << "m_ToMcoreInfo.De_RequestCounter_u32: " << m_ToMcoreInfo.De_RequestCounter_u32 
			<< "  m_ToMcoreInfo.De_CACC_Resume_u8" << (int)m_ToMcoreInfo.De_CACC_Resume_u8;
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_CC_REQ: {
		m_ToMcoreInfo.De_RequestCounter_u32 = m_Counter;
		auto request = msg.Deserialize< PadCACCCruiseControlRequestFrame>();
		m_ToMcoreInfo.De_CACC_Resume_u8 = true;
		m_ToMcoreInfo.De_Vset_f32 = request.data.setSpeed;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&m_ToMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		LOG(INFO) << "m_ToMcoreInfo.De_RequestCounter_u32: " << m_ToMcoreInfo.De_RequestCounter_u32
			<< "  m_ToMcoreInfo.De_CACC_Resume_u8" << (int)m_ToMcoreInfo.De_CACC_Resume_u8 
			<< "m_ToMcoreInfo.De_Vset_f32" << m_ToMcoreInfo.De_Vset_f32;
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_TIME_REQ: {
		m_ToMcoreInfo.De_RequestCounter_u32 = m_Counter;
		auto request = msg.Deserialize<PadCACCTimeRequestFrame>();
		m_ToMcoreInfo.De_TimeGapSet_f32 = request.data.timeHeadway;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&m_ToMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		LOG(INFO) << "m_ToMcoreInfo.De_RequestCounter_u32: " << m_ToMcoreInfo.De_RequestCounter_u32
		<< "  m_ToMcoreInfo.De_TimeGapSet_f32" << (int)m_ToMcoreInfo.De_TimeGapSet_f32;

	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_ON_REQ: {
		m_ToMcoreInfo.De_RequestCounter_u32 = m_Counter;
		m_ToMcoreInfo.De_IDA_Switch_u8= true;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&m_ToMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		LOG(INFO) << "m_ToMcoreInfo.De_RequestCounter_u32: " << m_ToMcoreInfo.De_RequestCounter_u32
			<< "  m_ToMcoreInfo.De_IDA_Switch_u8" << (int)m_ToMcoreInfo.De_IDA_Switch_u8;
	
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_OFF_REQ: {
		m_ToMcoreInfo.De_RequestCounter_u32 = m_Counter;
		m_ToMcoreInfo.De_IDA_Switch_u8 = false;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&m_ToMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		LOG(INFO) << "m_ToMcoreInfo.De_RequestCounter_u32: " << m_ToMcoreInfo.De_RequestCounter_u32
			<< "  m_ToMcoreInfo.De_IDA_Switch_u8" << (int)m_ToMcoreInfo.De_IDA_Switch_u8;

	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_MODE_REQ: {//该接口预留，暂不做
		LOG(INFO) << "De_CACC_SportMode reserve ";

#if 0
		is_SET_CACCmode_over = false;
		m_ToMcoreInfo.De_RequestCounter_u32 = m_Counter; 
		auto request = msg.Deserialize<PadCACCModeRequestFrame>();
		
		//signalInput.D = true;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&m_ToMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		
		PadCACCModeResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_MODE_RESP);
		if(0 == m_FromMcoreInfo.De_CACC_SportMode_Rsp_u8)
		{
			frame.rsp = m_FromMcoreInfo.De_CACC_SportMode_Rsp_u8;
			//未提供sportMode
			frame.data.currentSportMode = request.data.sportMode;
		}
		else
		{
			frame.rsp = m_FromMcoreInfo.De_CACC_SportMode_Rsp_u8;
			frame.data.currentSportMode = request.data.sportMode;
			LOG(INFO) << "De_CACC_SportMode fail ";
		}
		DoServerWrite(frame);
#endif
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_SENS_REQ: {
		m_ToMcoreInfo.De_RequestCounter_u32 = m_Counter; 
		auto request = msg.Deserialize<PadCollisionSensRequestFrame>();
		m_ToMcoreInfo.De_FCW_SnvtySet_u8 = request.data.collisionSensitivity;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&m_ToMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		LOG(INFO) << "m_ToMcoreInfo.De_RequestCounter_u32: " << m_ToMcoreInfo.De_RequestCounter_u32
			<< "  m_ToMcoreInfo.De_FCW_SnvtySet_u8" << (int)m_ToMcoreInfo.De_FCW_SnvtySet_u8;
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_ON_REQ: {//待确认5
		m_ToMcoreInfo.De_RequestCounter_u32 = m_Counter;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&m_ToMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		LOG(INFO) << "m_ToMcoreInfo.De_RequestCounter_u32: " << m_ToMcoreInfo.De_RequestCounter_u32;
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_OFF_REQ: {//待确认6
		m_ToMcoreInfo.De_RequestCounter_u32 = m_Counter;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&m_ToMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		LOG(INFO) << "m_ToMcoreInfo.De_RequestCounter_u32: " << m_ToMcoreInfo.De_RequestCounter_u32;
	}
		 break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_AEB_ON_REQ: {//待确认7
		m_ToMcoreInfo.De_AEB_SwtRequest_u8 = true;
		m_ToMcoreInfo.De_RequestCounter_u32 = m_Counter;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&m_ToMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		LOG(INFO) << "m_ToMcoreInfo.De_RequestCounter_u32: " << m_ToMcoreInfo.De_RequestCounter_u32
			<< "  m_ToMcoreInfo.De_AEB_SwtRequest_u8" << (int)m_ToMcoreInfo.De_AEB_SwtRequest_u8;
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_AEB_OFF_REQ: {//待确认8
		m_ToMcoreInfo.De_AEB_SwtRequest_u8 = false;
		m_ToMcoreInfo.De_RequestCounter_u32 = m_Counter; 
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&m_ToMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		LOG(INFO) << "m_ToMcoreInfo.De_RequestCounter_u32: " << m_ToMcoreInfo.De_RequestCounter_u32
			<< "  m_ToMcoreInfo.De_AEB_SwtRequest_u8" << (int)m_ToMcoreInfo.De_AEB_SwtRequest_u8;
	}
		break;

	default:
		m_PendingRequest = HMI_PAD_TAG_DEF::HMI_TAG_INVALID_REQ;
		LOG(INFO) << "unexpected tag from pad:" << static_cast<int>(tag);
		return;
	}	

    m_TimeoutTimer.expires_after(std::chrono::milliseconds(HMI_REQUEST_TIMEOUT));
    m_TimeoutTimer.async_wait(
		std::bind([this](HMI_PAD_TAG_DEF reqTag, uint32_t cnt)
	{
		if((this->m_PendingRequest == reqTag) && (this->m_Counter == cnt)) 
		{
			DoResponseError(m_PendingRequest, TIMEOUT);
			this->m_PendingRequest = HMI_PAD_TAG_DEF::HMI_TAG_INVALID_REQ;
		}
	}
	, m_PendingRequest
	, m_Counter)
	);
}

void HmiProxy::HandleRecieveIPCRespMsg()
{
	switch (m_PendingRequest)
	{
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_ON_REQ:
	{
		struct PadCACCOnResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_ON_RESP);
		if(m_Counter == (m_FromMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == m_FromMcoreInfo.De_CACC_CACCOn_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
			else if(false == m_FromMcoreInfo.De_CACC_CACCOn_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
				LOG(INFO) << "fail to open CACC";
			}
			else 
			{
				frame.rsp = RESP_MISMATCH;
				LOG(INFO) << "fail to open CACC";
			}
			DoServerWrite(frame);
		}
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_OFF_REQ:
	{
		struct PadCACCOffResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_OFF_RESP);
		if(m_Counter == (m_FromMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == m_FromMcoreInfo.De_CACC_CACCOff_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
			else if (0 == m_FromMcoreInfo.De_CACC_CACCOff_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
				LOG(INFO) << "fail to close CACC";
			}
			else
			{
				frame.rsp = RESP_MISMATCH;
				LOG(INFO) << "fail to close CACC";
			}
			DoServerWrite(frame);
		}
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_RESET_REQ:
	{
		struct PadCACCResetResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_RESET_RESP);
		if(m_Counter == (m_FromMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == m_FromMcoreInfo.De_CACC_CACCResume_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
			else if (0 == m_FromMcoreInfo.De_CACC_CACCResume_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
				LOG(INFO) << "fail to reset CACC";
			}
			else
			{
				frame.rsp = RESP_MISMATCH;
				LOG(INFO) << "fail to reset CACC";
			}
			DoServerWrite(frame);
		}
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_CC_REQ:
	{
		struct PadCACCCruiseControlResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_CC_RESP);
		if(m_Counter == (m_FromMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == m_FromMcoreInfo.De_CACC_SpeedSet_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
				frame.data.currentSpeed = m_ToMcoreInfo.De_Vset_f32;
			}
			else if (0 == m_FromMcoreInfo.De_CACC_SpeedSet_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
				LOG(INFO) << "fail to set speed ";
			}
			else
			{
				frame.rsp = RESP_MISMATCH;
				frame.data.currentSpeed = m_ToMcoreInfo.De_Vset_f32;
				LOG(INFO) << "fail to set speed ";
			}
			DoServerWrite(frame);
		}
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_TIME_REQ:
	{
		PadCACCTimeResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_TIME_RESP);
		if(m_Counter == (m_FromMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == m_FromMcoreInfo.De_CACC_TimeGapSet_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
				frame.data.currentTimeHeadway = m_ToMcoreInfo.De_TimeGapSet_f32;
			}
			else if (0 == m_FromMcoreInfo.De_CACC_TimeGapSet_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
				LOG(INFO) << "fail to set snvytime";
			}
			else
			{
				frame.rsp = RESP_MISMATCH;
				frame.data.currentTimeHeadway = m_ToMcoreInfo.De_TimeGapSet_f32;
				LOG(INFO) << "fail to set snvytime";
			}
			DoServerWrite(frame);
		}
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_ON_REQ:
	{
		struct PadIDAOnResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_ON_RESP);
		if(m_Counter == (m_FromMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == m_FromMcoreInfo.De_CACC_IDAOn_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
			else if (0 == m_FromMcoreInfo.De_CACC_IDAOn_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
				LOG(INFO) << "fail to open IDA";
			}
			else
			{
				frame.rsp = RESP_MISMATCH;
				LOG(INFO) << "fail to open IDA";
			}
			DoServerWrite(frame);
		}
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_OFF_REQ:
	{
		struct PadIDAOffResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_OFF_RESP);
		if(m_Counter == (m_FromMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == m_FromMcoreInfo.De_CACC_IDAOff_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
			else if (0 == m_FromMcoreInfo.De_CACC_IDAOff_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
				LOG(INFO) << "fail to close IDA";
			}
			else
			{
				frame.rsp = RESP_MISMATCH;
				LOG(INFO) << "fail to close IDA";
			}
			DoServerWrite(frame);
		}
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_MODE_REQ://暂不实现
	{
		struct PadCACCModeResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_MODE_RESP);
		if(m_Counter == (m_FromMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == m_FromMcoreInfo.De_CACC_SportMode_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
				//........
			}
			else if (0 == m_FromMcoreInfo.De_CACC_SportMode_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
				LOG(INFO) << "fail to set sport mode";
			}
			else
			{
				frame.rsp = RESP_MISMATCH;
				LOG(INFO) << "fail to set sport mode";
			}
			DoServerWrite(frame);
		}
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_SENS_REQ:
	{
		struct PadCollisionSensResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_SENS_RESP);
		if(m_Counter == (m_FromMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			frame.rsp = 0; //响应操作成功
			frame.data.currentCollisionSensitivity = m_ToMcoreInfo.De_FCW_SnvtySet_u8;
		}
		else 
		{
			frame.rsp = RESP_MISMATCH;
			LOG(INFO) << "fail to CAEB_COLLISION_SENS";
		}
		DoServerWrite(frame);
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_ON_REQ:
	{
		struct PadCollisionWarnOnResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_ON_RESP);
		if(m_Counter == (m_FromMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			//if(true == m_FromMcoreInfo.De_FCW_SnvtySet_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
		}
		else 
		{
			frame.rsp = RESP_MISMATCH;
			LOG(INFO) << "fail to CAEB_COLLISION_WARN_ON";
		}
		DoServerWrite(frame);
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_OFF_REQ:
	{
		struct PadCollisionWarnOffResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_OFF_RESP);
		if(m_Counter == (m_FromMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			//if(true == m_FromMcoreInfo.De_FCW_SnvtySet_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
		}
		else 
		{
			frame.rsp = RESP_MISMATCH;
		}
		DoServerWrite(frame);
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_AEB_ON_REQ:
	{
		struct PadAEBOnResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_AEB_ON_RESP);
		if(m_Counter == (m_FromMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			//if(true == m_FromMcoreInfo.De_FCW_SnvtySet_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
		}
		else 
		{
			frame.rsp = RESP_MISMATCH;
		}
		
		DoServerWrite(frame);
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_AEB_OFF_REQ:
	{
		struct PadAEBOffResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_AEB_OFF_RESP);
		if(m_Counter == (m_FromMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			//if(true == m_FromMcoreInfo.De_FCW_SnvtySet_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
		}
		else 
		{
			frame.rsp = RESP_MISMATCH;
		}
		
		DoServerWrite(frame);
	}
		break;
	default:
		return;
	}
	m_PendingRequest = HMI_PAD_TAG_DEF::HMI_TAG_INVALID_REQ;
}

void HmiProxy::DoResponseError(HMI_PAD_TAG_DEF reqTag, const int error)
{
	LOG(INFO) << "DoResponseError for Mcore Function timeout!!!!!";
	auto resp_tag = static_cast<int>(reqTag) + 1;
	switch (reqTag) {
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_ON_REQ:
	{
		PadCACCOnResponseFrame frame;
		frame.tag = resp_tag;
		frame.rsp = error;
		DoServerWrite(frame);
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_OFF_REQ:
	{
		PadCACCOffResponseFrame frame;
		frame.tag = resp_tag;
		frame.rsp = error;
		DoServerWrite(frame);
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_RESET_REQ:
	{
		PadCACCResetResponseFrame frame;
		frame.tag = resp_tag;
		frame.rsp = error;
		DoServerWrite(frame);
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_CC_REQ:
	{
		PadCACCCruiseControlResponseFrame frame;
		frame.tag = resp_tag;
		frame.rsp = error;
		DoServerWrite(frame);
		frame.data.currentSpeed = 0;
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_TIME_REQ:
	{
		PadCACCTimeResponseFrame frame;
		frame.tag = resp_tag;
		frame.rsp = error;
		DoServerWrite(frame);
		frame.data.currentTimeHeadway = 0;
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_ON_REQ:
	{
		PadIDAOnResponseFrame frame;
		frame.tag = resp_tag;
		frame.rsp = error;
		DoServerWrite(frame);
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_OFF_REQ:
	{
		PadIDAOffResponseFrame frame;
		frame.tag = resp_tag;
		frame.rsp = error;
		DoServerWrite(frame);
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_MODE_REQ://暂不实现
	{
		PadCACCModeResponseFrame frame;
		frame.tag = resp_tag;
		frame.rsp = error;
		DoServerWrite(frame);
		frame.data.currentSportMode = 0;
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_SENS_REQ:
	{
		PadCollisionSensResponseFrame frame;
		frame.tag = resp_tag;
		frame.rsp = error;
		DoServerWrite(frame);
		frame.data.currentCollisionSensitivity = 0;
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_ON_REQ:
	{
		PadCollisionWarnOnResponseFrame frame;
		frame.tag = resp_tag;
		frame.rsp = error;
		DoServerWrite(frame);
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_OFF_REQ:
	{
		PadCollisionWarnOffResponseFrame frame;
		frame.tag = resp_tag;
		frame.rsp = error;
		DoServerWrite(frame);
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_AEB_ON_REQ:
	{
		PadAEBOnResponseFrame frame;
		frame.tag = resp_tag;
		frame.rsp = error;
		DoServerWrite(frame);
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_AEB_OFF_REQ:
	{
		PadAEBOffResponseFrame frame;
		frame.tag = resp_tag;
		frame.rsp = error;
		DoServerWrite(frame);
	}
			break;
	default:
		break;
	}

}

void HmiProxy::SendCACCStatus()
{
	m_CACCStatus.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_STATUS_REP);
	DoServerWrite(m_CACCStatus);
	memset(&m_CACCStatus, 0, sizeof(PadCACCStatusFrame));
}

void HmiProxy::SendCAEBStatus()
{
	m_CAEBStatus.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_STATUS_REP);
	DoServerWrite(m_CAEBStatus);
	memset(&m_CAEBStatus, 0, sizeof(PadCAEBStatusFrame));
}

void HmiProxy::SendSysErrorStatus()
{
	m_SysErrorVec.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_SYS_ERROR_REP);
	DoServerWrite(m_SysErrorVec);
	memset(&m_SysErrorVec, 0, sizeof(PadSystemErrorVectorFrame));
}

void HmiProxy::SendHostVeh_CIPVInfo()
{

/*   数据精度未调整!!!!!!!!!!!!!!!!!!!
	//CIPV数据上报，周期性200ms定时上报
	bool cipvExist = false;
    for(uint32_t i = ADAS_GSENTRY_OBJ_VEHI_NUM; i < ADAS_GSENTRY_OBJ_VEHI_NUM + ADAS_CAMERA_OBJ_VEHI_NUM; i++)
	{
        auto obj = DataRepo::GetInstance().GetCddFusionData().cddObjects[i];
		if(obj.De_CIPV_u8 == true)
		{
			cipvExist = true;
			
			// 填充本车信息
			m_HostVehCIPVInfo.data.heading = m_HostVehInfo.data.heading;    //度  精度0.01
			m_HostVehCIPVInfo.data.latitude = m_HostVehInfo.data.latitude;  //度 [-90，90]
			m_HostVehCIPVInfo.data.longitude = m_HostVehInfo.data.longitude; //度 [-180，180]
			m_HostVehCIPVInfo.data.speed = m_HostVehInfo.data.speed;    //KM/H
			m_HostVehCIPVInfo.data.acceleration = m_HostVehInfo.data.acceleration;  //m/s^2  ,精度0.01
			//目标车车速（绝对速度）
			float r_vx = obj.De_vx_f32;
			float r_vy = obj.De_vy_f32;
			double r_v = sqrtf(powf(r_vx, 2) + powf(r_vy, 2));

			m_HostVehCIPVInfo.data.objSpeed = ADASMath::convertSpeedM2Km(r_v);       // =>目标车车速 单位km/h

			//目标距离（相对距离）
			float r_dx = obj.De_dx_f32;
			float r_dy = obj.De_dy_f32;
			double r_d = sqrtf(powf(r_dx, 2) + powf(r_dy, 2));
			m_HostVehCIPVInfo.data.followDistance = ADASMath::convertDistanceF2I(round(r_d * 100) / 100);   // m ,精度0.01

			if(r_vx < 0) //迎向 x为车辆前进方向
			{
				m_HostVehCIPVInfo.data.relativeVeloc = ADASMath::convertSpeedM2Km(r_v) + m_HostVehCIPVInfo.data.speed; 
				if (0 != m_HostVehCIPVInfo.data.relativeVeloc) 
				{
					m_HostVehCIPVInfo.data.objDistance =  ADASMath::convertDistanceI2F(m_HostVehCIPVInfo.data.followDistance) 
						/ ADASMath::convertSpeedKm2M(m_HostVehCIPVInfo.data.relativeVeloc);
				}
				else
				{
					m_HostVehCIPVInfo.data.objDistance = 0;
				}  
				    
			}
			else  //同向
			{
				m_HostVehCIPVInfo.data.relativeVeloc = m_HostVehCIPVInfo.data.speed - ADASMath::convertSpeedM2Km(r_v);    // =>相对速度 单位km/h
				if (0 != m_HostVehCIPVInfo.data.relativeVeloc)
				{
					m_HostVehCIPVInfo.data.objDistance =  ADASMath::convertDistanceI2F(m_HostVehCIPVInfo.data.followDistance)
						/ abs(ADASMath::convertSpeedKm2M(m_HostVehCIPVInfo.data.relativeVeloc));     // =>目标时距 单位s
				}
				else
				{
					m_HostVehCIPVInfo.data.objDistance = 0;
				}
			}
		} 
	}

	if(false == cipvExist)
	{
		LOG(INFO) <<"no cipv, send host data";
		m_HostVehInfo.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_EGO_VEH_DATA_REP);
		DoServerWrite(m_HostVehInfo);
		memset(&m_HostVehInfo, 0, sizeof(PadEgoVehInfoFrame));
	}
	else
	{
		LOG(INFO) <<"have cipv, send hostcipv data";
		m_HostVehCIPVInfo.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_EGO_VEH_DATA_REP);
		DoServerWrite(m_HostVehCIPVInfo);
		memset(&m_HostVehCIPVInfo, 0, sizeof(PadEgoVehCIPVInfoFrame));
	}
	*/

	// ##########临时方案，根据第一辆v2x目标填充CIPV上报##########

    auto v2xObj = DataRepo::GetInstance().GetV2xData().objVehicle[0];
	if(v2xObj.localId)
	{
		// 填充本车信息
		m_HostVehCIPVInfo.data.heading = m_HostVehInfo.data.heading;      //度  精度0.01
		m_HostVehCIPVInfo.data.latitude = m_HostVehInfo.data.latitude;   //度 [-90，90]
		m_HostVehCIPVInfo.data.longitude = m_HostVehInfo.data.longitude; //度 [-180，180]
		m_HostVehCIPVInfo.data.speed = m_HostVehInfo.data.speed;         //KM/H
		m_HostVehCIPVInfo.data.acceleration = m_HostVehInfo.data.acceleration;  //m/s^2  ,精度0.1

		//目标车车速（绝对速度）
		m_HostVehCIPVInfo.data.objSpeed = round(ADASMath::convertSpeedM2Km(ADASMath::convertSpeedI2F(v2xObj.speed)));       // =>目标车车速 单位km/h
		//目标距离（相对距离）
		PositionModel host_p;
		host_p.setLatitude(m_HostVehInfo.data.latitude);
		host_p.setLongitude(m_HostVehInfo.data.longitude);
		PositionModel obj_p;
		obj_p.setLatitude(ADASMath::convertDegLatLonI2F(v2xObj.vehicelPos.latitude));
		obj_p.setLongitude(ADASMath::convertDegLatLonI2F(v2xObj.vehicelPos.longitude));
		LOG(INFO) << "host position(degree): " << host_p.to_string() << "  obj position(degree): " << obj_p.to_string();
		m_HostVehCIPVInfo.data.followDistance = round(ADASMath::getDistance(host_p, obj_p) * 100) / 100;  // m ,精度0.01
		//时距  // =>目标时距 单位s
		if(0 != (abs(ADASMath::convertSpeedKm2M(m_HostVehCIPVInfo.data.speed) - ADASMath::convertSpeedI2F(v2xObj.speed))))
		{
			m_HostVehCIPVInfo.data.objDistance = round(m_HostVehCIPVInfo.data.followDistance / abs((ADASMath::convertSpeedKm2M(m_HostVehCIPVInfo.data.speed) 
				- ADASMath::convertSpeedI2F(v2xObj.speed))) * 100) / 100;
		}
		else
		{
			m_HostVehCIPVInfo.data.objDistance = 0.01;
		}
		m_HostVehCIPVInfo.data.relativeVeloc = round(abs(m_HostVehCIPVInfo.data.speed - ADASMath::convertSpeedM2Km(ADASMath::convertSpeedI2F(v2xObj.speed))));
		// =>相对速度 单位km/h
	
		LOG(INFO) <<"have v2xObj[0], send host v2xObj data";
		LOG(INFO) << "\n\tfor v2xObj[0]: "
              << "\n\tm_HostVehCIPVInfo.data.heading:       " << m_HostVehCIPVInfo.data.heading
			  << "\n\tm_HostVehCIPVInfo.data.latitude:      " << m_HostVehCIPVInfo.data.latitude
			  << "\n\tm_HostVehCIPVInfo.data.longitude:     " << m_HostVehCIPVInfo.data.longitude
			  << "\n\tm_HostVehCIPVInfo.data.speed:         " << m_HostVehCIPVInfo.data.speed
			  << "\n\tm_HostVehCIPVInfo.data.acceleration:  " << m_HostVehCIPVInfo.data.acceleration
			  << "\n\tm_HostVehCIPVInfo.data.objSpeed:      " << m_HostVehCIPVInfo.data.objSpeed
			  << "\n\tm_HostVehCIPVInfo.data.followDistance:" << m_HostVehCIPVInfo.data.followDistance
			  << "\n\tm_HostVehCIPVInfo.data.objDistance:   " << m_HostVehCIPVInfo.data.objDistance
			  << "\n\tm_HostVehCIPVInfo.data.relativeVeloc: " << m_HostVehCIPVInfo.data.relativeVeloc;
		m_HostVehCIPVInfo.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_EGO_VEH_DATA_REP);
		DoServerWrite(m_HostVehCIPVInfo);
		memset(&m_HostVehCIPVInfo, 0, sizeof(PadEgoVehCIPVInfoFrame));
	}
	else
	{
		LOG(INFO) <<"no v2xObj[0], send host data";
		m_HostVehInfo.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_EGO_VEH_DATA_REP);
		DoServerWrite(m_HostVehInfo);
		memset(&m_HostVehInfo, 0, sizeof(PadEgoVehInfoFrame));
	}
}

void HmiProxy::SendCACCDecisionInfo()
{
	m_CACCDecision.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_DECISION_REP);
	DoServerWrite(m_CACCDecision);
	memset(&m_CACCDecision, 0, sizeof(PadCACCDecisionFrame));
}

void HmiProxy::SendCAEBDecisionInfo()
{
	m_CAEBDecision.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_CAEB_REP);
	DoServerWrite(m_CAEBDecision);
	memset(&m_CAEBDecision, 0, sizeof(PadCAEBDecisionFrame));
}

void HmiProxy::SendSwitchSettings()
{
	m_SwitchSetingInfo.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_SWITCH_SETTING_REP);
	DoServerWrite(m_SwitchSetingInfo);
	memset(&m_SwitchSetingInfo, 0, sizeof(PadSwitchSettingsFrame));
}

void HmiProxy::SendV2XObjVehInfo()
{
	RemoteVehicleInfo tmpVel;
	for(uint32_t i = 0; i < ADAS_GSENTRY_OBJ_VEHI_NUM; i++)
	{
		auto v2xVel = DataRepo::GetInstance().GetV2xData().objVehicle[i];
		if(v2xVel.localId)
		{
			tmpVel.vehicleId = std::to_string(v2xVel.localId);
			tmpVel.speed = round(ADASMath::convertSpeedM2Km(ADASMath::convertSpeedI2F(v2xVel.speed)));
			tmpVel.acceleration = round(sqrtf(powf(ADASMath::convertAccI2F(v2xVel.accelSet.latitude), 2)
				+ powf(ADASMath::convertAccI2F(v2xVel.accelSet.longitude), 2)) * 10) / 10;
			tmpVel.latitude = ADASMath::convertDegLatLonI2F(v2xVel.vehicelPos.latitude);
			tmpVel.longitude = ADASMath::convertDegLatLonI2F(v2xVel.vehicelPos.longitude);
			tmpVel.heading = round(ADASMath::convertHeadingI2F(v2xVel.objectHeadingAngle) * 100) / 100;
			LOG(INFO) << "\n\ttmpVel.vehicleId " << tmpVel.vehicleId
				<< "\n\ttmpVel.speed:       "   << tmpVel.speed
				<< "\n\ttmpVel.acceleration " << tmpVel.acceleration
				<< "\n\ttmpVel.latitude:     " << tmpVel.latitude
				<< "\n\ttmpVel.longitude:    " << tmpVel.longitude
				<< "\n\ttmpVel.heading:      " << tmpVel.heading;
			m_V2xEgoVehInfo.data.remoteVehicleInfo.push_back(tmpVel);
		}
	}
	m_V2xEgoVehInfo.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_V2X_OBJ_REP);
	DoServerWrite(m_V2xEgoVehInfo);
	std::vector<RemoteVehicleInfo>().swap(m_V2xEgoVehInfo.data.remoteVehicleInfo);
}


