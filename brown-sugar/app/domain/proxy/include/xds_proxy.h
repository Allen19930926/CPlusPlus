#ifndef XDS_PROXY_H
#define XDS_PROXY_H

#include "iproxy.h"
#include "event_msg.h"
#include "xds_tcp_message.h"
#include "ipc_data.h"

class XdsProxy : public IProxy
{
public:
    XdsProxy(asio::io_context& ioService, short listenPort);
    ~XdsProxy() = default;
    void ProcessIncomingMessage(MsgType msgType, uint8_t* data, uint16_t len);
    void SendMessage();

private:
    virtual void Init() override;
    virtual void DoClientWrite(const char* buf , const uint16_t len) {}
    virtual void DoServerWrite(const char* buf , const uint16_t len) {}


    void WriteXYZ(float acc_x, float acc_y, float acc_z, float gyro_x, float fyro_y, float fyro_z, float pitchAngle, float rollAngle, float headingAngle, float velSpd);
    void WriteSTD(double latitude, double longtitude, float locatHeight, float locat);
    void WriteGPS(int gpsFlagPos, int numSV, int gpsFlagHead, int gpsAge, int carStatus, int status);
    void WriteUTC(int year, int month, int day, int hour, int min, int sec, int msec);  

private:
    AdasAsioTcpServer<XdsTcpMessage, XdsTcpMessage> server;
    IPC_GNSS_Data gnssData;
    IPC_GNSS_UTC utc;
    float vehSpeedMs;
};

#endif
