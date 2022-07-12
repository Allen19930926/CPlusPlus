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
    virtual void Start() override;
    void SetPeriodWriteTask(const uint32_t interval);

private:
    void ConstructServerAliveMsg();
    void ConstructClientAliveMsg();

private:
    AdasAsioTcpServer server;
    AdasAsioTcpClient client;
    std::string serverAliveMsg;
    std::string clientAliveMsg;
};

#endif /* C1F171C1_DCAD_43AA_A19A_966988C389AB */
