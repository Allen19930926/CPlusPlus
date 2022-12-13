#ifndef CDS_PROXY_H_
#define CDS_PROXY_H_

#include "nlohmann/json.hpp"
#include "iproxy.h"
#include "ipc_data.h"
#include "cdsproxy_tcp_message.h"

using nlohmann::json;

class CdsProxy : public IProxy
{
public:
    CdsProxy(asio::io_context& ioService, short port);
    ~CdsProxy() = default;

    /**
     * @name: ProcessIncomingMessage
     * @msg: handle IPC msg of CAN
     * @param {MsgType} msgType
     * @param {uint8_t*} data
     * @param {uint16_t} len
     * @return {void}
     */
    void ProcessIncomingMessage(MsgType msgType, uint8_t* data, uint16_t len);
    /**
     * @name: SendperiodMessage
     * @msg: send CAN data at intervals of 50ms
     * @return {void}
     */
    void SendperiodMessage_50ms();
private:
    virtual void Init() override;
    virtual void DoClientWrite(const char* buf , const uint16_t len) {}
    virtual void DoServerWrite(const char* buf , const uint16_t len);
    /**
     * @name: addMsgHeadAndSend
     * @msg: add protocol header for data and send
     * @param {char*} data
     * @param {unsigned int} len
     * @return {void}
     */
    void addMsgHeadAndSend(const char* data, unsigned int len);

private:
    AdasAsioTcpServer<CdsTcpMessage, CdsTcpMessage> m_Server;
    IPC_CAN_Data m_Cds_CanData;
};


#endif