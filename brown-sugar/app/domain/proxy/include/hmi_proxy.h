#ifndef C1F171C1_DCAD_43AA_A19A_966988C389AB
#define C1F171C1_DCAD_43AA_A19A_966988C389AB

#include "iproxy.h"
#include "hmi_tcp_message.h"
#include "ipc_data.h"
#include "hmi_pad_data.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

class HmiProxy : public IProxy
{
public:
    HmiProxy(asio::io_context& ioService, std::string clientIp, std::string clientPort, short listenPort);
    ~HmiProxy() = default;
    /**
     * @name: ProcessRecieveIPCData
     * @msg: handle ipc_msg
     * @param {MsgType} msgType
     * @param {uint8_t*} data
     * @param {uint16_t} len
     * @return {void}
     */
    void ProcessRecieveIPCData(MsgType msgType, uint8_t* data, uint16_t len);
    /**
     * @name: ProcessRecieveGsentryData
     * @msg: handle gSentry(HMI)_msg
     * @param {MsgType} msgType
     * @param {uint8_t*} data
     * @param {uint16_t} len
     * @return {void}
     */
    void ProcessRecieveGsentryData(MsgType msgType, uint8_t* data, uint16_t len);
    /**
     * @name: ProcessRecievePadData
     * @msg: handle PAD_msg
     * @param {MsgType} msgType
     * @param {uint8_t*} data
     * @param {uint16_t} len
     * @return {void}
     */
    void ProcessRecievePadData(MsgType msgType, uint8_t* data, uint16_t len);
    /**
     * @name: DoPeriodTask_200ms
     * @msg: do task at intervals of 200ms
     * @return {void}
     */
    void DoPeriodTask_200ms();
    /**
     * @name: DoPeriodTask_50ms
     * @msg: do task at intervals of 50ms
     * @return {void}
     */
    void DoPeriodTask_50ms();

private:
    /**
     * @name: Init
     * @msg: hmiproxy module init
     * @return {void}
     */
    virtual void Init() override;
    /**
     * @name: KeepAlive
     * @msg: register keepalive_function
     * @param {uint32_t} interval: interval time(ms)
     * @return {void}
     */
    void KeepAlive(const uint32_t interval);
    /**
     * @name: SendClientAliveMsg
     * @msg: client send keepalive_package to gSentry(HMI)
     * @return {void}
     */
    void SendClientAliveMsg();
    /**
     * @name: getCorrectJsonString
     * @msg: add '\0' at the tail of JSONString which does not have '\0' at the tail 
     * @param {char*} param  JSONString which does not have '\0' at the tail
     * @param {uint16_t} len JSONString length
     * @return {std::string}
     */
    std::string getCorrectJsonString(const char* param, const uint16_t len);
    /**
     * @name: DoClientWrite
     * @msg: client send data
     * @param {HmiTcpMessage} msg
     * @return {void}
     */
    void DoClientWrite(HmiTcpMessage& msg);
    /**
     * @name: DoServerWrite
     * @msg: server send data
     * @param {HmiTcpMessage} msg
     * @return {void}
     */
    void DoServerWrite(HmiTcpMessage& msg);

    virtual void DoClientWrite(const char* buf , const uint16_t len) override {}
    virtual void DoServerWrite(const char* buf , const uint16_t len) override {}
    
    template <typename T>
    void DoServerWrite(T& msg);
    
    /**
     * @name: HandlegSentryMessage
     * @msg: handle gsentry(HMI)_data
     * @param {uint8_t*} data
     * @param {uint16_t} len
     * @return {void}
     */
    void HandlegSentryMessage(uint8_t* data, uint16_t len);
    /**
     * @name: ProcessCAEBCollisionWarning
     * @msg: select FCW of V2V eventtype and delete FCW type
     * @param {json&} j
     * @return {void}
     */
    void ProcessCAEBCollisionWarning(json& j);

    /**
     * @name: HandlePadMessage
     * @msg: handle PAD_data
     * @param {uint8_t*} data
     * @param {uint16_t} len
     * @return {void}
     */
    void HandlePadMessage(uint8_t* data, uint16_t len);
    /**
     * @name: HandleRecieveIPCRespMsg
     * @msg: receive IPC_msg and send to PAD
     * @return {void}
     */
    void HandleRecieveIPCRespMsg();
    /**
     * @name: DoResponseError
     * @msg: handle error or timeout response and send to PAD
     * @param {HMI_PAD_TAG_DEF} reqTag
     * @param {int} error
     * @return {*}
     */
    void DoResponseError(HMI_PAD_TAG_DEF reqTag, const int error);
    /**
     * @name: SendCACCStatus
     * @msg: send CACC status when status change
     * @return {void}
     */
    void SendCACCStatus();
    /**
     * @name: SendCAEBStatus
     * @msg: send CAEB status when status change
     * @return {void}
     */
    void SendCAEBStatus();
    /**
     * @name: SendSysErrorStatus
     * @msg: send system error at intervals of 50ms, send NULL when no error
     * @return {void}
     */
    void SendSysErrorStatus();
    /**
     * @name: SendHostVeh_CIPVInfo
     * @msg: send host Veh and CIPV info at intervals of 200ms
     * @return {void}
     */
    void SendHostVeh_CIPVInfo();
    /**
     * @name: SendCACCDecisionInfo
     * @msg: send CACC decision when event happen
     * @return {void}
     */
    void SendCACCDecisionInfo();
    /**
     * @name: SendCAEBDecisionInfo
     * @msg: send CAEB decision at intervals of 50ms
     * @return {void}
     */
    void SendCAEBDecisionInfo(); 
    /**
     * @name: SendSwitchSettings
     * @msg: send switchSetting Info when system start for the first time
     * @return {void}
     */
    void SendSwitchSettings();
    /**
     * @name: SendV2XObjVehInfo
     * @msg: send all V2X obj_Vehs intervals of 200ms
     * @return {*}
     */
    void SendV2XObjVehInfo();

private:
    AdasAsioTcpServer<HmiTcpMessage, HmiTcpMessage> m_Server;
    AdasAsioTcpClient<HmiTcpMessage, HmiTcpMessage> m_Client;

    PadCACCStatusFrame m_CACCStatus;           // CACC状态
    PadCAEBStatusFrame m_CAEBStatus;           // CAEB状态
    PadSystemErrorVectorFrame m_SysErrorVec;   // 系统错误信息
    PadEgoVehCIPVInfoFrame m_HostVehCIPVInfo;  // 车辆信息上报 2004
    PadV2XEgoVehInfoFrame m_V2xEgoVehInfo;     // v2x的所有原车
    PadCACCDecisionFrame m_CACCDecision;       // CACC决策
    PadCAEBDecisionFrame m_CAEBDecision;       // CAEB决策
    PadSwitchSettingsFrame m_SwitchSetingInfo; // 功能开关状态
    IPC_System_Error m_IPC_systemError;        // 接受M核错误信息

    FuncCoord_FAM_HMI_Info m_FromMcoreInfo;     // 接受M核FCW
    SignalInput_HMI_BUS m_ToMcoreInfo;          // 发给M核信息
    uint32_t m_Counter = 0;                     // pad控制类消息计数

    HMI_PAD_TAG_DEF m_PendingRequest = HMI_PAD_TAG_DEF::HMI_TAG_INVALID_REQ;  // pad控制类消息
    asio::steady_timer m_TimeoutTimer;          // pad控制类请求超时处理定时器
};
#endif /* C1F171C1_DCAD_43AA_A19A_966988C389AB */
