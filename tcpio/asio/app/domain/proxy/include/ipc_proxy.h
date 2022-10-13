#ifndef IPC_PROXY_H
#define IPC_PROXY_H

#include "ipcm.h"
#include "iproxy.h"
#include "event_msg.h"


class IpcProxy : public IProxy
{
public:
    IpcProxy(asio::io_context& io);
    ~IpcProxy();

    void ProcessIncomingMessage(MsgType msgType, uint8_t* data, uint16_t len);
    void DoPeriodTask_50ms();
    
private:
    virtual void Init() override;
    virtual void DoClientWrite(const char* buf , const uint16_t len) {}
    virtual void DoServerWrite(const char* buf , const uint16_t len) {}

    

private:
    IpcM ipcm;
};



#endif
