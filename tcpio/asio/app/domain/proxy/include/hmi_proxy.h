#ifndef C1F171C1_DCAD_43AA_A19A_966988C389AB
#define C1F171C1_DCAD_43AA_A19A_966988C389AB

#include "iproxy.h"
#include "nlohmann/nlohmann/json.hpp"

using json = nlohmann::json;

#define PAD_SERVER_PORT 52133

class HmiProxy : public IProxy
{
public:
    HmiProxy(asio::io_context& ioService, std::string clientIp, std::string clientPort, short listenPort);
    ~HmiProxy() = default;

private:
    virtual void Init() override;
    virtual void DoClientWrite(const char* buf , const uint16_t len);
    virtual void DoServerWrite(const char* buf , const uint16_t len);

private:
    void KeepAlive(const uint32_t interval);
    void WriteServerAliveMsg();
    void WriteClientAliveMsg();

private:
    AdasAsioTcpServer<DefaultChatMessage, DefaultChatMessage> server;
    AdasAsioTcpClient<DefaultChatMessage, DefaultChatMessage> client;
};

#endif /* C1F171C1_DCAD_43AA_A19A_966988C389AB */
