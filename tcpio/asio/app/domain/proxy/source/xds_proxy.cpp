#include "xds_proxy.h"
#include <map>
#include <glog/logging.h>

const std::map<MsgType, size_t> XdsProxy_MessageDefinition = {
    {MsgType::IPC_GNSS_ACC,  sizeof(IPC_GNSS_Acc)},
    {MsgType::IPC_GNSS_GYRO,  sizeof(IPC_GNSS_Gyro)},
    {MsgType::IPC_GNSS_HEADING_PITCH_ROLL, sizeof(IPC_GNSS_HeadingPitchRoll)},
    {MsgType::IPC_GNSS_HEIGHT_TIME, sizeof(IPC_GNSS_HeightAndTime)},
    {MsgType::IPC_GNSS_LAT_LONG, sizeof(IPC_GNSS_LatitudeLongitude)},
    {MsgType::IPC_GNSS_SPEED, sizeof(IPC_GNSS_Speed)},
    {MsgType::IPC_GNSS_DATA_INFO, sizeof(IPC_GNSS_DataInfo)},
    {MsgType::IPC_GNSS_STD, sizeof(IPC_GNSS_Std)},
	{MsgType::IPC_GNSS_UTC, sizeof(IPC_GNSS_UTC)},
};

XdsProxy::XdsProxy(asio::io_context& ioService, short listenPort)
            : IProxy(ioService, MsgType::XDS_DUMMY), server(ioService, MsgType::XDS_DUMMY, listenPort)
{
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
	case MsgType::IPC_GNSS_ACC: {
		IPC_GNSS_Acc* tempAcc = (IPC_GNSS_Acc *)(data);
		acc.acc_x = tempAcc->acc_x;
		acc.acc_y = tempAcc->acc_y;
		acc.acc_z = tempAcc->acc_z;
		LOG(INFO) << "Acc received";
	}
		break;
	case MsgType::IPC_GNSS_GYRO:{
		IPC_GNSS_Gyro* tempGyro = (IPC_GNSS_Gyro*)(data);
		gyro.gyro_x = tempGyro->gyro_x;
		gyro.gyro_y = tempGyro->gyro_y;
		gyro.gyro_z = tempGyro->gyro_z;
		LOG(INFO) << "Gyro received";
	}
		break;
	case MsgType::IPC_GNSS_HEADING_PITCH_ROLL:{
		IPC_GNSS_HeadingPitchRoll* tempHeading = (IPC_GNSS_HeadingPitchRoll*)(data);
		headingpitchroll.pitchAngle = tempHeading->pitchAngle;
		headingpitchroll.rollAngle = tempHeading->rollAngle;
		headingpitchroll.headingAngle = tempHeading->headingAngle;
		LOG(INFO) << "HeadingPitchRoll received";
	}
		break;
	case MsgType::IPC_GNSS_HEIGHT_TIME:{
		IPC_GNSS_HeightAndTime* tempHeight = (IPC_GNSS_HeightAndTime*)(data);
		heightandtime.locatHeight = tempHeight->locatHeight;
		heightandtime.time = tempHeight->time;
		LOG(INFO) << "HeightTime received";
	}
		break;
	case MsgType::IPC_GNSS_LAT_LONG:{
		IPC_GNSS_LatitudeLongitude* tempLat = (IPC_GNSS_LatitudeLongitude*)(data);
		latitudelongitude.latitude = tempLat->latitude;
		latitudelongitude.longitude = tempLat->longitude;
		LOG(INFO) << "LatLong received";
	}
		break;
	case MsgType::IPC_GNSS_SPEED:{
		IPC_GNSS_Speed* tempSpeed = (IPC_GNSS_Speed*)(data);
		speed.northSpd = tempSpeed->northSpd;
		speed.eastSpd = tempSpeed->eastSpd;
		speed.toGroundSpd = tempSpeed->toGroundSpd;
		LOG(INFO) << "Speed received";
	}
		break;
	case MsgType::IPC_GNSS_DATA_INFO:{
		datainfo = *(IPC_GNSS_DataInfo*)(data);
		LOG(INFO) << "GNSSData received";
	}
		break;	
	case MsgType::IPC_GNSS_STD:{
		IPC_GNSS_Std* tempStd = (IPC_GNSS_Std*)(data);
		std.stdLat = tempStd->stdLat;
		std.stdLon = tempStd->stdLat;
		std.stdLocateHeight = tempStd->stdLat;
		std.stdHeading = tempStd->stdLat;
		LOG(INFO) << "GNSSStd received";
	}
		break;
	case MsgType::IPC_GNSS_UTC:{
		utc = *(IPC_GNSS_UTC*)(data);
		LOG(INFO) << "GNSSSUTC received";
	}
		break;
	default:
		break;
	}
}

void XdsProxy::WriteXYZ(float acc_x, float acc_y, float acc_z, float gyro_x, float fyro_y, float fyro_z,\
                            float pitchAngle, float rollAngle, float headingAngle, float velSpd)   //CANid 0x500 0x501 0x502
{
    char xyzbuf[160];
    sprintf(xyzbuf, "$GNSSINS570,XYZ,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n", acc_x, acc_y, acc_z, gyro_x, fyro_y, fyro_z, \
            pitchAngle, rollAngle, headingAngle, velSpd);
    server.write(XdsTcpMessage(xyzbuf));
}

void XdsProxy::WriteGPS(int gpsFlagPos, int numSV, int gpsFlagHead, int gpsAge, int carStatus, int status)   //CANid 0x506
{
    char gpsbuf[80];
    sprintf(gpsbuf, "$GNSSINS570,GPS,%d,%d,%d,%d,%d,%d\r\n", gpsFlagPos, numSV, gpsFlagHead, gpsAge, carStatus, status);
    server.write(XdsTcpMessage(gpsbuf));
}

void XdsProxy::WriteSTD(float latitude, float longtitude, float locatHeight, float locat)    //CANid 0x503 0x504 0x505
{
    char stdbuf[80];
    sprintf(stdbuf, "$GNSSINS570,STD,%f,%f,%f,%f\r\n", latitude, longtitude, locatHeight, locat);
    server.write(XdsTcpMessage(stdbuf));
}

void XdsProxy::WriteUTC(int year, int month, int day, int hour, int min, int sec, int msec)  //CANid 0x508
{
    char utcbuf[80];
    sprintf(utcbuf, "$GNSSINS570,UTC,%d,%d,%d,%d,%d,%d,%d\r\n", year, month, day, hour, min, sec, msec);
    server.write(XdsTcpMessage(utcbuf));
}

void XdsProxy::SendMessage() {
	WriteXYZ(acc.acc_x, acc.acc_y, acc.acc_z, gyro.gyro_x, gyro.gyro_y, gyro.gyro_z, headingpitchroll.pitchAngle, headingpitchroll.rollAngle, headingpitchroll.headingAngle, 80);   
	WriteGPS(datainfo.gpsFlagPos, datainfo.numSv, datainfo.gpsFlagHeading, datainfo.gpsAge, datainfo.gpsStatus, datainfo.status);
	WriteSTD(latitudelongitude.latitude, latitudelongitude.longitude, std.stdLocateHeight, 0.2);
	WriteUTC(utc.utcYear, utc.utcMonth, utc.utcDay, utc.utcHour, utc.utcMin, utc.utcSec, utc.utcMilliSec);
}
