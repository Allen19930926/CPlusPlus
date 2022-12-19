#ifndef A436CFB7_7668_4BA2_A18D_440C2DACEC50
#define A436CFB7_7668_4BA2_A18D_440C2DACEC50

#include "domain/data_preprocessor/data_preprocessor.h"
#include "domain/data_preprocessor/prescan_receiver.h"
#include "infrastructure/tcp/fusion_asio_tcp_client.h"
#include "infrastructure/tcp/fusion_asio_tcp_server.h"
#include "infrastructure/udp/fusion_asio_udp_server.h"

class DataReceiver
{
public:
    DataReceiver(asio::io_context& ioService);
    ~DataReceiver();
    void Start();
private:
    void ProcessReadMsg(FusionChatMessage msg);
    void PrecessV2xMsg(char* data, uint32_t len);
    void PrecessCameraMsg(char* data, uint32_t len);
private:
    void ProcessPrescanCamera(char* data, uint32_t len);
    void ProcessPrescanRadar(char* data, uint32_t len);
private:
    DataPreprocessor       process;
    PrescanReceiver        prescan_process;
    FusionAsioTcpClient    client;
    FusionAsioTcpServer    server;
    FusionAsioUdpServer    prescan_camera_server;
    FusionAsioUdpServer    prescan_radar_server;
};

#endif /* A436CFB7_7668_4BA2_A18D_440C2DACEC50 */
