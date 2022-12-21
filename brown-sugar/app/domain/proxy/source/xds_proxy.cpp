#include "xds_proxy.h"
#include "event_msg.h"
#include "ipc_data.h"
#include <ios>
#include <map>
#include <glog/logging.h>
#include <sstream>
#include <iomanip>

const std::map<MsgType, size_t> XdsProxy_MessageDefinition = {
    {MsgType::IPC_GNSS_DATA,  sizeof(IPC_GNSS_Data)},
	{MsgType::IPC_GNSS_UTC, sizeof(IPC_GNSS_UTC)},
	{MsgType::IPC_EVH, sizeof(EVH_SubjectInfo_BUS)},
};

XdsProxy::XdsProxy(asio::io_context& ioService, short listenPort)
            : IProxy(ioService, MsgType::XDS_DUMMY), server(ioService, MsgType::XDS_DUMMY, listenPort)
{
	vehSpeedMs = 0;
	memset(&gnssData, 0, sizeof(IPC_GNSS_Data));
	memset(&utc, 0, sizeof(utc));
}


void XdsProxy::Init() {
	server.start();
}

void XdsProxy::ProcessIncomingMessage(MsgType msgType, uint8_t * data, uint16_t len) {
	auto iter = XdsProxy_MessageDefinition.find(msgType);
	if (iter == XdsProxy_MessageDefinition.end())
	{
		LOG(INFO) << "undefined message type " << int(msgType);
		return;
	}
	if (iter->second != len)
	{
		LOG(INFO) << "mismatched message length" << int(msgType);
		return;
	}
	switch (msgType)
	{
	case MsgType::IPC_EVH:
	{
		EVH_SubjectInfo_BUS* hostInfo = (EVH_SubjectInfo_BUS*)(data);
		vehSpeedMs = hostInfo->De_ego_vxMs_f32;
	}
	    break;
	case MsgType::IPC_GNSS_DATA:
	{
		gnssData = *(IPC_GNSS_Data *)(data);
	}
	break;
	case MsgType::IPC_GNSS_UTC:{
		utc = *(IPC_GNSS_UTC*)(data);
		WriteUTC(utc.utcYear, utc.utcMonth, utc.utcDay, utc.utcHour, utc.utcMin, utc.utcSec, utc.utcMilliSec);
		// LOG(INFO) << "GNSSSUTC received";
	}
		break;
	default:
		break;
	}
}

void XdsProxy::WriteXYZ(float acc_x, float acc_y, float acc_z, float gyro_x, float fyro_y, float fyro_z,\
                            float pitchAngle, float rollAngle, float headingAngle, float velSpd)   //CANid 0x500 0x501 0x502
{
	static int counter = 0;
	std::ostringstream oss;
	oss << "$GNSSINS570,XYZ," 
		<< std::fixed << std::setprecision(7) << acc_x << "," 
		<< std::fixed << std::setprecision(7) << acc_y << "," 
		<< std::fixed << std::setprecision(7) << acc_z << "," 
		<< std::fixed << std::setprecision(7) << gyro_x << ","  
		<< std::fixed << std::setprecision(7) << fyro_y << "," 
		<< std::fixed << std::setprecision(7) << fyro_z  << ","
	    << std::fixed << std::setprecision(7) << pitchAngle << "," 
		<< std::fixed << std::setprecision(7) << rollAngle << "," 
		<< std::fixed << std::setprecision(7) << headingAngle << "," 
		<< std::fixed << std::setprecision(7) << velSpd  << "\r\n";
	auto msg = XdsTcpMessage(oss.str());
    server.write(msg);
	if (counter++ == 20)
	{
		LOG(INFO) << oss.str();
		counter = 0;
	}
}

void XdsProxy::WriteGPS(int gpsFlagPos, int numSV, int gpsFlagHead, int gpsAge, int carStatus, int status)   //CANid 0x506
{
	static int counter = 0;
	std::ostringstream oss;
    oss << "$GNSSINS570,GPS," << gpsFlagPos << "," << numSV << "," << gpsFlagHead << "," << gpsAge << "," << carStatus << "," << status << "\r\n";
	auto msg = XdsTcpMessage(oss.str());
    server.write(msg);
	if (counter++ == 20)
	{
		LOG(INFO) << oss.str();
		counter = 0;
	}
}

void XdsProxy::WriteSTD(double latitude, double longtitude, float locatHeight, float locat)    //CANid 0x503 0x504 0x505
{
	static int counter = 0;
	std::ostringstream oss;
    oss << "$GNSSINS570,STD," << std::fixed << std::setprecision(8) << latitude << "," << std::fixed << std::setprecision(8) << longtitude << "," << locatHeight << "," << locat << "\r\n";
	auto msg = XdsTcpMessage(oss.str());
    server.write(msg);
	if (counter++ == 20)
	{
		LOG(INFO) << oss.str();
		counter = 0;
	}
}

void XdsProxy::WriteUTC(int year, int month, int day, int hour, int min, int sec, int msec)  //CANid 0x508
{
	static int counter = 0;
	std::ostringstream oss;
    oss << "$GNSSINS570,UTC," << year << "," << month << "," << day << "," << hour << "," << min << "," << sec << "," << msec << "\r\n";
	auto msg = XdsTcpMessage(oss.str());
    server.write(msg);
	if (counter++ == 5)
	{
		LOG(INFO) << oss.str();
		counter = 0;
	}
}

void XdsProxy::SendMessage() {
	WriteXYZ(gnssData.acc_x, gnssData.acc_y, gnssData.acc_z, gnssData.gyro_x, gnssData.gyro_y, gnssData.gyro_z, gnssData.pitchAngle, gnssData.rollAngle, gnssData.headingAngle, vehSpeedMs);   
	WriteGPS(gnssData.gpsFlagPos, gnssData.numSv, gnssData.gpsFlagHeading, gnssData.gpsAge, gnssData.gpsStatus, gnssData.status);
	WriteSTD(gnssData.latitude, gnssData.longitude, gnssData.stdLocateHeight, 0.2);

}
