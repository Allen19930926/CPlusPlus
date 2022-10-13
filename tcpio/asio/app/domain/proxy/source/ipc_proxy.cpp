#include "ipc_proxy.h"
#include "event_msg.h"
#include "ipc_data.h"
#include "iproxy.h"
#include <map>

const std::map<MsgType, size_t> IpcProxy_MessageDefinition = {
    {MsgType::IPC_HMI_CTRL, sizeof(SignalInput_HMI_BUS)},
    {MsgType::IPC_OBJ_INFO, sizeof(CDD_Fusion_ObjInfo_Array40)},
    {MsgType::IPC_LANE_INFO, sizeof(CDD_Fusion_LaneInfo_Array4)},
    {MsgType::IPC_GSENTRY_WARN, sizeof(CDD_gSentry_WarningInfo_BUS)},
    {MsgType::IPC_DIS2ENDLANE, sizeof(CDD_DistanceToEndLine)},
    {MsgType::IPC_TRAFFIC_LIGHT_INFO, sizeof(CDD_CurntLaneTrafficLightInfo_BUS)},
};

IpcProxy::IpcProxy(asio::io_context& io) : IProxy(io, MsgType::IPC_DUMMY){

}

IpcProxy::~IpcProxy() {
	ipcm.DeInit();
}


void IpcProxy::Init() {
    ipcm.Init();
}

void IpcProxy::DoPeriodTask_50ms() {
    ipcm.Run();
}

void IpcProxy::ProcessIncomingMessage(MsgType msgType, uint8_t * data, uint16_t len) {
	auto iter = IpcProxy_MessageDefinition.find(msgType);
	if (iter == IpcProxy_MessageDefinition.end())
	{
		std::cout << "undefined message type" << std::endl;
		return;
	}
	if (iter->second != len)
	{
		std::cout << "mismatched message length" << int(msgType) << std::endl;
		return;
	}
	switch (msgType)
	{
	case MsgType::IPC_HMI_CTRL:
		ipcm.Write(IPC_APPL_FRAME_HMI_CTRL, data, len);
		break;
	case MsgType::IPC_OBJ_INFO:
		ipcm.Write(IPC_APPL_FRAME_FUSION_OBJ, data, len);
		break;
	case MsgType::IPC_LANE_INFO:
		ipcm.Write(IPC_APPL_FRAME_FUSION_LANE_INFO, data, len);
		break;
	case MsgType::IPC_GSENTRY_WARN:
		ipcm.Write(IPC_APPL_FRAME_GSENTRY_WARN_INFO, data, len);
		break;
	case MsgType::IPC_DIS2ENDLANE:
		ipcm.Write(IPC_APPL_FRAME_DIST_TO_END_LANE, data, len);
		break;
	case MsgType::IPC_TRAFFIC_LIGHT_INFO:
		ipcm.Write(IPC_APPL_FRAME_TRAFFIC_LIGHT_INFO, data, len);
		break;
	default:
		break;
	}
}