#include <cstdint>
#include <math.h>
#include <cstring>

#include "asio/steady_timer.hpp"
#include "event_msg.h"
#include "hmi_proxy.h"
#include "hmi_gSentry.h"
#include "hmi_pad_data.h"
#include "hmi_message.h"
#include "hmi_tcp_message.h"
#include "ipc_data.h"
#include "event_queue.h"
#include <glog/logging.h>
#include <sstream>
#include <string>
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
	{MsgType::IPC_SYS_ERROR,  sizeof(IPC_System_Error)},
	{MsgType::IPC_EVH,  sizeof(EVH_SubjectInfo_BUS)},
	{MsgType::IPC_HMI_INFO,  sizeof(FuncCoord_FAM_HMI_Info)},
};

HmiProxy::HmiProxy(asio::io_context& ioService, std::string clientIp, std::string clientPort, short listenPort)
            : IProxy(ioService, MsgType::HMI_DUMMY), server(ioService, MsgType::HMI_PAD, listenPort),
              client(ioService, MsgType::HMI_GSENTRY, clientIp, clientPort),
			  timeoutTimer(ioService)
{
	std::memset(&systemError, 0, sizeof(systemError));
	pad_CACCStatus.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_STATUS_REP);
	pad_CAEBStatus.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_STATUS_REP);
	pad_SysErrorVec.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_SYS_ERROR_REP);
	pad_HostVehInfo.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_EGO_VEH_DATA_REP);
	pad_HostVehCIPVInfo.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_EGO_VEH_DATA_REP);
	pad_CACCDecision.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_DECISION_REP);
	pad_CAEBDecision.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_CAEB_REP);
	pad_SwitchSetingInfo.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_SWITCH_SETTING_REP);

}

void HmiProxy::Init()
{
    server.start();
    client.start();
    KeepAlive(1000);	
}

void HmiProxy::KeepAlive(const uint32_t interval)
{
    SetPeriodTask(interval, std::bind(&HmiProxy::SendClientAliveMsg, this));
}

std::string HmiProxy::ConstructServerAliveMsg()
{
	//pad发保活已在HandlePadMessage（）完成回复
    return "";
}

void HmiProxy::SendClientAliveMsg()
{
    gSentryKeepAliveRequestFrame frame;
    frame.tag = static_cast<int>(HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_KEEP_ALIVE_REQ);
    json j;
    to_json(j, frame);
    client.write(j.dump());
}

void HmiProxy::DoClientWrite(HmiTcpMessage msg)
{
    client.write(msg);
}

void HmiProxy::DoServerWrite(HmiTcpMessage msg)
{
    // LOG(INFO) << "send data to pad: " << std::string(reinterpret_cast<char const*>(msg.Body()), msg.BodyLength());
    server.write(msg);
}

template <typename T>
void HmiProxy::DoServerWrite(T& msg)
{
	HmiTcpMessage tcpMsg = HmiTcpMessage(HmiMessage(msg));
    // LOG(INFO) << ": " << std::string(reinterpret_cast<char const*>(tcpMsg.Body()), tcpMsg.BodyLength());
    server.write(tcpMsg);
}


void HmiProxy::ProcessRecieveIPCData(MsgType msgType, uint8_t* data, uint16_t len)
{
	// LOG(INFO) << "receive data from IPC: " << static_cast<int>(msgType);
	auto iter = HmiProxy_MessageDefinition.find(msgType);
	if (iter == HmiProxy_MessageDefinition.end())
	{
		LOG(INFO) << "undefined message type " << int(msgType);
		return;
	}
	if (iter->second != len)
	{
		LOG(INFO) << "mismatched message length" << int(msgType);
		return;
	}
	switch (msgType)
	{
	case MsgType::IPC_GNSS_DATA:
	{
		IPC_GNSS_Data gnssData = *(IPC_GNSS_Data *)(data);
		pad_HostVehInfo.data.heading = round(gnssData.headingAngle * 100) / 100;
		pad_HostVehInfo.data.latitude = gnssData.latitude;
		pad_HostVehInfo.data.longitude = gnssData.longitude;
	}; break;
	case MsgType::IPC_SYS_ERROR:
	{
		pad_SysErrorVec.data.systemError.clear();
		systemError = *(IPC_System_Error *)data;
		for (int i = 0; i < IPC_MAX_SYSTEM_ERROR_NUM; i++) {
			if (systemError.De_Fim_Fids_u8[i] != 0) {
				PadSystemErrorFrame p;
				p.type = systemError.De_Fim_Fids_u8[i];
				p.detail = "";
				pad_SysErrorVec.data.systemError.push_back(p);
			}
		}
	}; break;

	case MsgType::IPC_EVH:
	{
		EVH_SubjectInfo_BUS* hostInfo = (EVH_SubjectInfo_BUS*)(data);
		pad_HostVehInfo.data.speed = round(hostInfo->De_ego_vxMs_f32);
		pad_HostVehInfo.data.acceleration = round(sqrtf(powf(hostInfo->De_ego_axMs2_f32, 2) + powf(hostInfo->De_ego_ayMs2_f32, 2)) * 10) / 10;
	}; break;

	case MsgType::IPC_HMI_INFO: 
	{
		FuncCoord_FAM_HMI_Info* tmp_getMcoreInfo = (FuncCoord_FAM_HMI_Info *)(data);
			
		if(tmp_getMcoreInfo->De_CACC_CACCStatus_u8 != getMcoreInfo.De_CACC_CACCStatus_u8 || 
		   tmp_getMcoreInfo->De_CACC_IDAStatus_u8 != getMcoreInfo.De_CACC_IDAStatus_u8)
		{
			//1、CACC功能状态上报,状态改变时上报(ACCStatus ,可选ACCfaultDetail, IDAStatus, 可选IDAfaultDetail, 可选reson)

			pad_CACCStatus.data.ACCStatus = tmp_getMcoreInfo->De_CACC_CACCStatus_u8;
			pad_CACCStatus.data.ACCfaultDetail = 0;
			pad_CACCStatus.data.IDAStatus = tmp_getMcoreInfo->De_CACC_IDAStatus_u8; 
			pad_CACCStatus.data.IDAfaultDetail = 0;
			pad_CACCStatus.data.reason = " ";
			SendCACCStatus();
		}
			
		if(tmp_getMcoreInfo->De_FCW_AEB_FuncSts_u8 != getMcoreInfo.De_FCW_AEB_FuncSts_u8)
		{
			//3\CAEB功能状态上报,状态改变时上报(CAEBStatus, 可选：faultDetail)
			pad_CAEBStatus.data.CAEBStatus = tmp_getMcoreInfo->De_FCW_AEB_FuncSts_u8;
			pad_CAEBStatus.data.faultDetail = "";
			SendCAEBStatus();
		}

		if (tmp_getMcoreInfo->De_CACC_Decision_u8 != getMcoreInfo.De_CACC_Decision_u8) {
			//CACC系统决策上报
			if (getMcoreInfo.De_CACC_Decision_u8 >= 20) {
				std::stringstream fmt;
				fmt << "绿波通行，车速为" << getMcoreInfo.De_CACC_Decision_u8 << "KPH";
				pad_CACCDecision.data.CACCDecision =  fmt.str();
			} else if (getMcoreInfo.De_CACC_Decision_u8 > 0 && getMcoreInfo.De_CACC_Decision_u8 <=3) {
				auto iter = HmiProxy_CACCDecisionDefinition.find(getMcoreInfo.De_CACC_Decision_u8);
				pad_CACCDecision.data.CACCDecision =  iter->second;
			}
			SendCACCDecisionInfo();
		}
		

    	//CAEB前向碰撞预警决策上报
		pad_CAEBDecision.data.preWarning = tmp_getMcoreInfo->De_FCW_AEB_FuncSts_u8;
		pad_CAEBDecision.data.CAEB = tmp_getMcoreInfo->De_AEB_Triger_u8;
    	
		getMcoreInfo = *tmp_getMcoreInfo;

		//对pad的请求进行响应
		HandleRecieveIPCRespMsg();
		
	}
		break;
		
	
	default:
		break;
	}
}


void HmiProxy::HandleRecieveIPCRespMsg()
{
	switch (pendingRequest)
	{
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_ON_REQ:
	{
		struct PadCACCOnResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_ON_RESP);
		if(counter == (getMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == getMcoreInfo.De_CACC_CACCOn_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
			else if (0 == getMcoreInfo.De_CACC_CACCOn_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
			}
			else 
			{
				frame.rsp = getMcoreInfo.De_CACC_CACCOn_Rsp_u8;
			}
		}
		else {
			frame.rsp = RESP_MISMATCH;
		}
		DoServerWrite(frame);
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_OFF_REQ:
	{
		struct PadCACCOffResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_OFF_RESP);
		if(counter == (getMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == getMcoreInfo.De_CACC_CACCOff_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
			else if (0 == getMcoreInfo.De_CACC_CACCOff_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
			}
			else
			{
				frame.rsp = getMcoreInfo.De_CACC_CACCOff_Rsp_u8;
			}
		}
		else {
			frame.rsp = RESP_MISMATCH;
		}
		DoServerWrite(frame);
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_RESET_REQ:
	{
		struct PadCACCResetResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_RESET_RESP);
		if(counter == (getMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == getMcoreInfo.De_CACC_CACCResume_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
			else if (0 == getMcoreInfo.De_CACC_CACCResume_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
			}
			else
			{
				frame.rsp = getMcoreInfo.De_CACC_CACCResume_Rsp_u8;
				LOG(INFO) << "fail to resume CACC";
			}
		}
		else 
		{
			frame.rsp = RESP_MISMATCH;
		}
		DoServerWrite(frame);
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_CC_REQ:
	{
		struct PadCACCCruiseControlResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_CC_RESP);
		if(counter == (getMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == getMcoreInfo.De_CACC_SpeedSet_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
				frame.data.currentSpeed = sendMcoreInfo.De_Vset_f32;
			}
			else if (0 == getMcoreInfo.De_CACC_SpeedSet_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
			}
			else
			{
				frame.rsp = getMcoreInfo.De_CACC_SpeedSet_Rsp_u8;
				frame.data.currentSpeed = sendMcoreInfo.De_Vset_f32;
				LOG(INFO) << "fail to set speed ";
			}
		}
		else 
		{
			frame.rsp = RESP_MISMATCH;
		}
		DoServerWrite(frame);
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_TIME_REQ:
	{
		PadCACCTimeResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_TIME_RESP);
		if(counter == (getMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == getMcoreInfo.De_CACC_TimeGapSet_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
				frame.data.currentTimeHeadway = sendMcoreInfo.De_TimeGapSet_f32;
			}
			else if (0 == getMcoreInfo.De_CACC_TimeGapSet_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
			}
			else
			{
				frame.rsp = getMcoreInfo.De_CACC_TimeGapSet_Rsp_u8;
				frame.data.currentTimeHeadway = sendMcoreInfo.De_TimeGapSet_f32;
				LOG(INFO) << "fail to set snvytime";
			}
		}
		else 
		{
			frame.rsp = RESP_MISMATCH;
		}
		DoServerWrite(frame);
	}
		break;


	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_ON_REQ:
	{
		struct PadIDAOnResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_ON_RESP);
		if(counter == (getMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == getMcoreInfo.De_CACC_IDAOn_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
			else if (0 == getMcoreInfo.De_CACC_IDAOn_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
			}
			else
			{
				frame.rsp = getMcoreInfo.De_CACC_IDAOn_Rsp_u8;
				LOG(INFO) << "fail to open IDA";
			}
		}
		else 
		{
			frame.rsp = RESP_MISMATCH;
		}
		DoServerWrite(frame);
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_OFF_REQ:
	{
		struct PadIDAOffResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_OFF_RESP);
		if(counter == (getMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == getMcoreInfo.De_CACC_IDAOff_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
			}
			else if (0 == getMcoreInfo.De_CACC_IDAOff_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
			}
			else
			{
				frame.rsp = getMcoreInfo.De_CACC_IDAOff_Rsp_u8;
				LOG(INFO) << "fail to close IDA";
			}
		}
		else 
		{
			frame.rsp = RESP_MISMATCH;
		}
		DoServerWrite(frame);
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_MODE_REQ://暂不实现
	{
		struct PadCACCModeResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_MODE_RESP);
		if(counter == (getMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			if(true == getMcoreInfo.De_CACC_SportMode_Rsp_u8)
			{
				frame.rsp = 0; //响应操作成功
				//........
			}
			else if (0 == getMcoreInfo.De_CACC_SportMode_Rsp_u8)
			{
				frame.rsp = GENERAL_FAIL;
			}
			else
			{
				frame.rsp = getMcoreInfo.De_CACC_SportMode_Rsp_u8;
				LOG(INFO) << "fail to set sport mode";
			}
		}
		else 
		{
			frame.rsp = RESP_MISMATCH;
		}
		DoServerWrite(frame);
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_SENS_REQ:
	{
		struct PadCollisionSensResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_SENS_RESP);
		if(counter == (getMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			frame.rsp = 0; //响应操作成功
			frame.data.currentCollisionSensitivity = sendMcoreInfo.De_FCW_SnvtySet_u8;
		}
		else 
		{
			frame.rsp = RESP_MISMATCH;
		}
		DoServerWrite(frame);
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_ON_REQ:
	{
		struct PadCollisionWarnOnResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_ON_RESP);
		if(counter == (getMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			//if(true == getMcoreInfo.De_FCW_SnvtySet_u8)
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

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_OFF_REQ:
	{
		struct PadCollisionWarnOffResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_OFF_RESP);
		if(counter == (getMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			//if(true == getMcoreInfo.De_FCW_SnvtySet_u8)
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
		if(counter == (getMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			//if(true == getMcoreInfo.De_FCW_SnvtySet_u8)
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
		if(counter == (getMcoreInfo.De_ResponseCounter_u32 - 1))
		{
			//if(true == getMcoreInfo.De_FCW_SnvtySet_u8)
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
	pendingRequest = HMI_PAD_TAG_DEF::HMI_TAG_INVALID_REQ;
}

void HmiProxy::ProcessRecieveGsentryData(MsgType msgType, uint8_t* data, uint16_t len)
{
	if(msgType == MsgType::HMI_GSENTRY)
	{
		HandlegSentryMessage(data, len);
	}
	else
	{
		LOG(INFO) << "No SentryMessage! ";
	}
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
        return ;
    }
	
	HMI_GSENTRY_TAG_DEF tag = HMI_GSENTRY_TAG_DEF(j["tag"]);
	switch(tag)
	{
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_CON_RESP: 
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
			DoServerWrite(HmiTcpMessage(HmiMessage(data, len)));
		}
		break;
		
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_CAEB_COLLISION_SENS_REP:{
			 /* Process Adas caution attention */
        	ProcessCAEBCollisionWarning(j);
			LOG(INFO) << "hmiproxy_client receive gSentry msg_tag is " << static_cast<int>(tag);
			DoServerWrite(HmiTcpMessage(HmiMessage(data, len)));
		}

		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_V2IWARNING:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_RLVWINFO:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_VRUCWINFO:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_TRAFFIClIGHT:
		case HMI_GSENTRY_TAG_DEF::GSENTRY_HMI_SLWINFO:{

			DoServerWrite(HmiTcpMessage(HmiMessage(data, len)));
		}
		break;

		default:
		LOG(INFO) << "unexpected tag from gsentry:" << static_cast<int>(tag);
		break;
	}
   
}


void HmiProxy::ProcessCAEBCollisionWarning(json& j)
{
    const uint32_t FCW_WARN = 1;
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
            fcwExist = getMcoreInfo.De_gSentry_Warning_u8 == 1 ? false : true;
            fcwIndex = i;
        }
    }

    if (fcwExist)
    {
        warnNode.erase(fcwIndex);
    }
    
}

void HmiProxy::ProcessRecievePadData(MsgType msgType, uint8_t* data, uint16_t len)
{
	if(msgType == MsgType::HMI_PAD)
	{
		HandlePadMessage(data, len);
	}
	else
	{
		LOG(INFO) << "Not PadMessage! ";
	}
}

void HmiProxy::DoResponseError(HMI_PAD_TAG_DEF reqTag, int error) {
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

//server interfsace
void HmiProxy::HandlePadMessage(uint8_t* data, uint16_t len) {
    LOG(INFO) << "received data from pad: " << std::string(reinterpret_cast<char const*>(data), len);
	HmiMessage msg(data, len);
	HMI_PAD_TAG_DEF tag = HMI_PAD_TAG_DEF(msg.GetTag());
	if (pendingRequest != HMI_PAD_TAG_DEF::HMI_TAG_INVALID_REQ) {
		DoResponseError(tag, SERVER_BUSY);
		pendingRequest = HMI_PAD_TAG_DEF::HMI_TAG_INVALID_REQ;
		return;
	}
	pendingRequest = tag;
	counter++;
	switch (tag) {
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_ON_REQ: {
		sendMcoreInfo.De_RequestCounter_u32 = counter; 
		sendMcoreInfo.De_CACC_Switch_u8 = true;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&sendMcoreInfo , sizeof(SignalInput_HMI_BUS)});
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_OFF_REQ: {
		sendMcoreInfo.De_RequestCounter_u32 = counter; 
		sendMcoreInfo.De_CACC_Switch_u8 = false;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&sendMcoreInfo , sizeof(SignalInput_HMI_BUS)});
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_RESET_REQ: {
		sendMcoreInfo.De_RequestCounter_u32 = counter; 
		sendMcoreInfo.De_CACC_Resume_u8 = true;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&sendMcoreInfo , sizeof(SignalInput_HMI_BUS)});
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_CC_REQ: {
		sendMcoreInfo.De_RequestCounter_u32 = counter; 
		auto request = msg.Deserialize< PadCACCCruiseControlRequestFrame>();
		sendMcoreInfo.De_CACC_Resume_u8 = true;
		sendMcoreInfo.De_Vset_f32 = request.data.setSpeed;                             
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&sendMcoreInfo , sizeof(SignalInput_HMI_BUS)});
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_TIME_REQ: {
		sendMcoreInfo.De_RequestCounter_u32 = counter; 
		auto request = msg.Deserialize<PadCACCTimeRequestFrame>();
		sendMcoreInfo.De_TimeGapSet_f32 = request.data.timeHeadway;        
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&sendMcoreInfo , sizeof(SignalInput_HMI_BUS)});
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_ON_REQ: {
		sendMcoreInfo.De_RequestCounter_u32 = counter; 
		sendMcoreInfo.De_IDA_Switch_u8= true;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&sendMcoreInfo , sizeof(SignalInput_HMI_BUS)});
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_IDA_OFF_REQ: {
		sendMcoreInfo.De_RequestCounter_u32 = counter; 
		sendMcoreInfo.De_IDA_Switch_u8 = false;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&sendMcoreInfo , sizeof(SignalInput_HMI_BUS)});
	}
		break;

	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_MODE_REQ: {//该接口预留，暂不做
		LOG(INFO) << "De_CACC_SportMode reserve ";

#if 0
		is_SET_CACCmode_over = false;
		sendMcoreInfo.De_RequestCounter_u32 = counter; 
		auto request = msg.Deserialize<PadCACCModeRequestFrame>();    
		
		//signalInput.D = true;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&sendMcoreInfo , sizeof(SignalInput_HMI_BUS)});
		
		PadCACCModeResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CACC_MODE_RESP);
		if(0 == getMcoreInfo.De_CACC_SportMode_Rsp_u8)
		{
			frame.rsp = getMcoreInfo.De_CACC_SportMode_Rsp_u8;
			//未提供sportMode
			frame.data.currentSportMode = request.data.sportMode;
		}
		else
		{
			frame.rsp = getMcoreInfo.De_CACC_SportMode_Rsp_u8;
			frame.data.currentSportMode = request.data.sportMode;
			LOG(INFO) << "De_CACC_SportMode fail ";
		}
		DoServerWrite(frame);
#endif
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_SENS_REQ: {
		sendMcoreInfo.De_RequestCounter_u32 = counter; 
		auto request = msg.Deserialize<PadCollisionSensRequestFrame>();
		sendMcoreInfo.De_FCW_SnvtySet_u8 = request.data.collisionSensitivity;
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&sendMcoreInfo , sizeof(SignalInput_HMI_BUS)});
	}
															 break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_ON_REQ: {//待确认5
		//signalInput.De_ = true;
		sendMcoreInfo.De_RequestCounter_u32 = counter; 
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&sendMcoreInfo , sizeof(SignalInput_HMI_BUS)});
	}
																break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_COLLISION_WARN_OFF_REQ: {//待确认6
		//signalInput.De_CACCSWOn_u8 = true;
		sendMcoreInfo.De_RequestCounter_u32 = counter; 
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&sendMcoreInfo , sizeof(SignalInput_HMI_BUS)});
	}
																 break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_AEB_ON_REQ: {//待确认7
		sendMcoreInfo.De_AEB_SwtRequest_u8 = true;
		sendMcoreInfo.De_RequestCounter_u32 = counter; 
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&sendMcoreInfo , sizeof(SignalInput_HMI_BUS)});
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CAEB_AEB_OFF_REQ: {//待确认8
		sendMcoreInfo.De_AEB_SwtRequest_u8 = false;
		sendMcoreInfo.De_RequestCounter_u32 = counter; 
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_HMI_CTRL, (const char *)&sendMcoreInfo , sizeof(SignalInput_HMI_BUS)});
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_KEEP_ALIVE_REQ: {
		LOG(INFO) << "keep alive request is received";
		PadKeepAliveResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_KEEP_ALIVE_RESP);
		frame.rsp = 0;
		frame.data.deviceNum = "CDC1X32";
		DoServerWrite(frame);
	}
		break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CON_REQ: {
		PadConnectResponseFrame frame;
		frame.tag = static_cast<int>(HMI_PAD_TAG_DEF::HMI_TAG_PAD_CON_RESP);
		frame.rsp = 0;
		frame.data.deviceSerialNum = "CDC1X32";
		DoServerWrite(frame);

		//2\ADAS系统开关状态上报，系统每次开机需主动上报（CACCSwitch, IDASwitch, currentTimeHeadway, CAEBWarningSwitch, currentCollisionSensitivity, CAEBSwitch）
		pad_SwitchSetingInfo.data.CACCSwitch = 0;
		pad_SwitchSetingInfo.data.IDASwitch = 0; 
		pad_SwitchSetingInfo.data.currentTimeHeadway = 0;
		pad_SwitchSetingInfo.data.CAEBWarningSwitch = 0;
		pad_SwitchSetingInfo.data.currentCollisionSensitivity = 0;
		pad_SwitchSetingInfo.data.CAEBSwitch = 0;
		SendSwitchSettings(); 
	}
	    break;
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_INIT_REQ:
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CLOSEV2X_REQ:
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_OPENV2X_REQ:
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_CLOSEONEV2X_REQ:
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_OPENONEV2X_REQ:
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_SENSE_REQ:
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_GSENTRY_STATUS_RESP:
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_V2X_STATUS_REQ:
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_V2XSWITCH_STATUS_REQ:
	case HMI_PAD_TAG_DEF::HMI_TAG_PAD_SENSESTATUS_REQ:{
		pendingRequest = HMI_PAD_TAG_DEF::HMI_TAG_INVALID_REQ;
		DoClientWrite(HmiTcpMessage(msg));
	}
		break;	

	default:
		pendingRequest = HMI_PAD_TAG_DEF::HMI_TAG_INVALID_REQ;
		LOG(INFO) << "unexpected tag from pad:" << static_cast<int>(tag);
		return;
	}	

    timeoutTimer.expires_after(std::chrono::milliseconds(HMI_REQUEST_TIMEOUT));
    timeoutTimer.async_wait(std::bind([this](HMI_PAD_TAG_DEF reqTag, uint32_t cnt){
		if (this->pendingRequest == reqTag && this->counter == cnt) {
			DoResponseError(pendingRequest, TIMEOUT);
			this->pendingRequest = HMI_PAD_TAG_DEF::HMI_TAG_INVALID_REQ;
		}
	}, pendingRequest, counter));
	
}

void HmiProxy::DoPeriodTask_50ms()
{
	//系统故障状态上报，有故障时50ms上报一次，无故障时发送空消息告诉hmi故障清除
	SendSysErrorStatus();
    // CAEB决策 50ms上报一次
    SendCAEBDecisionInfo(); 
}

void HmiProxy::DoPeriodTask_200ms()
 {

	//CIPV数据上报，周期性200ms定时上报
	bool cipvExist = false;
    for(uint32_t i = ADAS_GSENTRY_OBJ_VEHI_NUM; i < ADAS_GSENTRY_OBJ_VEHI_NUM + ADAS_CAMERA_OBJ_VEHI_NUM; i++)
	{
        auto obj = DataRepo::GetInstance().GetCddFusionData().cddObjects[i];
		if(obj.De_CIPV_u8 == true)
		{
			cipvExist = true;
			
			pad_HostVehCIPVInfo.data.heading = pad_HostVehInfo.data.heading;
			pad_HostVehCIPVInfo.data.latitude = pad_HostVehInfo.data.latitude;
			pad_HostVehCIPVInfo.data.longitude = pad_HostVehInfo.data.longitude;
			pad_HostVehCIPVInfo.data.speed = pad_HostVehInfo.data.speed;
			pad_HostVehCIPVInfo.data.acceleration = pad_HostVehInfo.data.acceleration;
			//目标车车速（绝对速度）
			float r_vx = obj.De_vx_f32;
			float r_vy = obj.De_vy_f32;
			double r_v = sqrtf(powf(r_vx, 2) + powf(r_vy, 2));

			pad_HostVehCIPVInfo.data.objSpeed = r_v / 3.6;       // =>目标车车速 单位km/h

			//目标距离（相对距离）
			float r_dx = obj.De_dx_f32;
			float r_dy = obj.De_dy_f32;
			double r_d = sqrtf(powf(r_dx, 2) + powf(r_dy, 2));
			pad_HostVehCIPVInfo.data.followDistance = round(r_d*100)/100;

			if(r_vx < 0) //迎向 x为车辆前进方向
			{
				pad_HostVehCIPVInfo.data.relativeVeloc = (r_v + pad_HostVehCIPVInfo.data.speed) / 3.6 ;    
				pad_HostVehCIPVInfo.data.objDistance =  r_d / (r_v + pad_HostVehCIPVInfo.data.speed);    
			}
			else  //同向
			{
				double tmp = (pad_HostVehCIPVInfo.data.speed - r_v) / 3.6;
				pad_HostVehCIPVInfo.data.relativeVeloc = (tmp > 0)? tmp : 0 ;    // =>相对速度 单位km/h
				pad_HostVehCIPVInfo.data.objDistance =  r_d / tmp;      // =>目标时距 单位s
			}
		} 
	}
	if(false == cipvExist)
	{
		DoServerWrite(pad_HostVehInfo);
	}
	else
	{
		DoServerWrite(pad_HostVehCIPVInfo);
	}

 }

void HmiProxy::SendCACCStatus()
{
	DoServerWrite(pad_CACCStatus);
}

void HmiProxy::SendCAEBStatus()
{
	DoServerWrite(pad_CAEBStatus);
}

void HmiProxy::SendSysErrorStatus()
{
	// auto errorInfo = systemError;
	// for (int i = 0; i < IPC_MAX_SYSTEM_ERROR_NUM; i++) {
	// 	if (errorInfo.De_Fim_Fids_u8[i] != 0) {
	// 		PadSystemErrorFrame p;
	// 		p.type = errorInfo.De_Fim_Fids_u8[i];
	// 		p.detail = "";
	// 		errorFrame.data.systemError.push_back(p);
	// 	}
	// }
	DoServerWrite(HmiTcpMessage(HmiMessage(pad_SysErrorVec)));
}
  
void HmiProxy::SendPadEgoVehInfo()
{
	DoServerWrite(pad_HostVehInfo);
}

void HmiProxy::SendCACCDecisionInfo()
{
	DoServerWrite(pad_CACCDecision);
}

void HmiProxy::SendCAEBDecisionInfo()
{
	DoServerWrite(pad_CAEBDecision);
}

void HmiProxy::SendSwitchSettings()
{
	DoServerWrite(pad_SwitchSetingInfo);
}


