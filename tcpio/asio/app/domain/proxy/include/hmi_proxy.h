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
    void KeepAlive(const uint32_t interval);
    virtual void DoPeriodClientWriteTask(PeriodTimer timer ,const uint32_t interval, std::string msg) override;
    virtual void DoPeriodServerWriteTask(PeriodTimer timer ,const uint32_t interval, std::string msg) override;
    std::string ConstructServerAliveMsg();
    std::string ConstructClientAliveMsg();

private:
    AdasAsioTcpServer server;
    AdasAsioTcpClient client;
};

#endif /* C1F171C1_DCAD_43AA_A19A_966988C389AB */
