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
    void ProcessIncomingMessage(MsgType msgType, uint8_t* data, uint16_t len);
    void SendperiodMessage();
private:
    virtual void Init() override;
    virtual void DoClientWrite(const char* buf , const uint16_t len) {}
    virtual void DoServerWrite(const char* buf , const uint16_t len);

    void addMsgHeadAndSend(const char* data, unsigned int len);

    uint64_t getSytemTime();


private:
    AdasAsioTcpServer<CdsTcpMessage, CdsTcpMessage> server;
    IPC_CAN_Data cds_CanData;
};


#endif