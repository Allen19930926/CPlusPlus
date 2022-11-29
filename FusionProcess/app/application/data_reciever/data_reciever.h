#ifndef A436CFB7_7668_4BA2_A18D_440C2DACEC50
#define A436CFB7_7668_4BA2_A18D_440C2DACEC50

#include "domain/data_preprocessor/data_preprocessor.h"
#include "infrastructure/tcp/fusion_asio_tcp_client.h"

class DataReciever
{
public:
    DataReciever(asio::io_context& ioService, std::string ipAddr, std::string port);
    ~DataReciever();
    void Start();
private:
    void ProcessReadMsg(FusionChatMessage msg);
    void PrecessV2xMsg(char* data, uint32_t len);
    void PrecessCameraMsg(char* data, uint32_t len);
private:
    DataPreprocessor       process;
    FusionAsioTcpClient    client;
};

#endif /* A436CFB7_7668_4BA2_A18D_440C2DACEC50 */
