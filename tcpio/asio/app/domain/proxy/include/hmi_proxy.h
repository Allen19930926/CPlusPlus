#ifndef C1F171C1_DCAD_43AA_A19A_966988C389AB
#define C1F171C1_DCAD_43AA_A19A_966988C389AB

#include <map>
#include "ipc_data.h"
#include "iproxy.h"
#include "hmi_message.h" 
#include "hmi_tcp_message.h"
#include "nlohmann/json.hpp"
#include "ipc_data.h"
#include "hmi_pad_data.h"



using json = nlohmann::json;


class HmiProxy : public IProxy
{
public:
    HmiProxy(asio::io_context& ioService, std::string clientIp, std::string clientPort, short listenPort);
    ~HmiProxy() = default;

    void ProcessRecieveIPCData(MsgType msgType, uint8_t* data, uint16_t len);
    void ProcessRecieveGsentryData(MsgType msgType, uint8_t* data, uint16_t len);
    void ProcessRecievePadData(MsgType msgType, uint8_t* data, uint16_t len);
    void DoPeriodTask_200ms();
    void DoPeriodTask_50ms();


private:
    virtual void Init() override;
    void KeepAlive(const uint32_t interval);
    std::string ConstructServerAliveMsg();
    void SendClientAliveMsg();

    virtual void DoClientWrite(const char* buf , const uint16_t len) {}
    virtual void DoServerWrite(const char* buf , const uint16_t len) {}
    void DoClientWrite(HmiTcpMessage msg);
    void DoServerWrite(HmiTcpMessage msg);
    
    template <typename T>
    void DoServerWrite(T& msg);
    
    //client
    void HandlegSentryMessage(uint8_t* data, uint16_t len);
    void ProcessCAEBCollisionWarning(json& j);

    //server
    void HandlePadMessage(uint8_t* data, uint16_t len);
    void HandleRecieveIPCRespMsg();
    void DoResponseError(HMI_PAD_TAG_DEF reqTag, int error);

    //CACC功能状态上报,状态改变时上报
    void SendCACCStatus();

    //CAEB功能状态上报,状态改变时上报
    void SendCAEBStatus();

    //系统故障状态上报，有故障时50ms上报一次，无故障时发送空消息告诉hmi故障清除
    void SendSysErrorStatus();

    //本车数据上报，周期性50ms定时上报
    void SendPadEgoVehInfo();

    //CACC系统决策上报
    void SendCACCDecisionInfo();

    //CAEB前向预警决策上报
    void SendCAEBDecisionInfo(); 

    //ADAS系统开关状态上报，系统每次开机需主动上报
    void SendSwitchSettings();

    unsigned long getTimestamp()
    {
	    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

private:
    AdasAsioTcpServer<HmiTcpMessage, HmiTcpMessage> server;
    AdasAsioTcpClient<HmiTcpMessage, HmiTcpMessage> client;

    PadCACCStatusFrame pad_CACCStatus;
    PadCAEBStatusFrame pad_CAEBStatus;
    PadSystemErrorVectorFrame pad_SysErrorVec;
    PadEgoVehInfoFrame pad_HostVehInfo;
    PadEgoVehCIPVInfoFrame pad_HostVehCIPVInfo;
    PadCACCDecisionFrame pad_CACCDecision;
    PadCAEBDecisionFrame pad_CAEBDecision;
    PadSwitchSettingsFrame pad_SwitchSetingInfo;
    IPC_System_Error systemError;
    

    FuncCoord_FAM_HMI_Info getMcoreInfo;  //接受M核FCW
    SignalInput_HMI_BUS sendMcoreInfo;         //发给M核
    
    uint32_t counter = 0;               // pad控制类消息计数

    HMI_PAD_TAG_DEF pendingRequest = HMI_PAD_TAG_DEF::HMI_TAG_INVALID_REQ;  //pad控制类消息
    bool requestTimeout = false;
    asio::steady_timer timeoutTimer;
};
#endif /* C1F171C1_DCAD_43AA_A19A_966988C389AB */
