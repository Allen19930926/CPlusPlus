#ifndef HMI_PAD_DATA_H_
#define HMI_PAD_DATA_H_

#include "nlohmann/json.hpp"
using json = nlohmann::json;

//pad与hmiproxy交互请求响应tag
enum class HMI_PAD_TAG_DEF: int {  
  HMI_TAG_INVALID_REQ = 0,

  HMI_TAG_PAD_CON_REQ = 3,            // pad连接请求
  HMI_TAG_PAD_CON_RESP = 4,           // pad连接响应
  HMI_TAG_PAD_INIT_REQ = 5,           // pad初始化请求（实际上pad不会与hmi发起连接，而是返回gSentry信息）
  HMI_TAG_PAD_CLOSEV2X_REQ = 9,       // pad关闭所有V2X功能请求
  HMI_TAG_PAD_OPENV2X_REQ = 11,       // pad开启所有V2X功能请求
  HMI_TAG_PAD_CLOSEONEV2X_REQ = 13,   // pad关闭单项V2X功能请求
  HMI_TAG_PAD_OPENONEV2X_REQ = 15,    // pad开启单项V2X功能请求
  HMI_TAG_PAD_SENSE_REQ = 17,         //pad设置gentry灵敏度请求
  HMI_TAG_PAD_GSENTRY_STATUS_RESP = 20, //pad对（19）v2x/gsentry状态请求响应（图标显示需求）
  HMI_TAG_PAD_V2X_STATUS_REQ = 21,      //pad查询v2x/gsentry状态请求
  HMI_TAG_PAD_V2XSWITCH_STATUS_REQ = 23,     //pad查询gV2X场景开关状态请求     
  HMI_TAG_PAD_SENSESTATUS_REQ = 25,          //pad获取gentry灵敏度请求

  HMI_TAG_PAD_CACC_STATUS_REP = 2001, // ADAS系统发送CACC功能状态的请求
  HMI_TAG_PAD_CAEB_STATUS_REP = 2002, // ADAS系统发送CAEB功能状态的请求
  HMI_TAG_PAD_SYS_ERROR_REP = 2003, // ADAS系统发送系统故障的请求
  HMI_TAG_PAD_EGO_VEH_DATA_REP = 2004, // ADAS系统上报本车数据请求
  HMI_TAG_PAD_CACC_DECISION_REP = 2005, // ADAS系统上报CACC系统决策数据请求
  HMI_TAG_PAD_CAEB_CAEB_REP = 2006, // ADAS系统上报CAEB预警信息及CAEB激活报警碰撞信息请求
  HMI_TAG_PAD_SWITCH_SETTING_REP = 2007, // ADAS系统上报CAEB自动紧急制动信息
  HMI_TAG_PAD_V2X_OBJ_REP = 2008, // ADAS系统上报v2x原车信息
  HMI_TAG_PAD_CACC_ON_REQ = 3001, // 客户端开启CACC功能请求
  HMI_TAG_PAD_CACC_ON_RESP = 3002, // 客户端开启CACC功能响应
  HMI_TAG_PAD_CACC_OFF_REQ = 3003, // 客户端关闭CACC功能请求
  HMI_TAG_PAD_CACC_OFF_RESP = 3004, // 客户端关闭CACC功能响应
  HMI_TAG_PAD_CACC_RESET_REQ = 3005, // 客户端重置CACC功能请求
  HMI_TAG_PAD_CACC_RESET_RESP = 3006, // 客户端重置CACC功能响应
  HMI_TAG_PAD_CACC_CC_REQ = 3007, // 客户端请求设置CACC巡航速度
  HMI_TAG_PAD_CACC_CC_RESP = 3008, // 客户端设置CACC巡航速度响应
  HMI_TAG_PAD_CACC_TIME_REQ = 3009, // 客户端请求设置CACC时距
  HMI_TAG_PAD_CACC_TIME_RESP = 3010, // 客户端设置CACC时距响应
  HMI_TAG_PAD_CACC_IDA_ON_REQ = 3011, // 客户端开启IDA功能请求
  HMI_TAG_PAD_CACC_IDA_ON_RESP = 3012, // 客户端开启IDA功能响应
  HMI_TAG_PAD_CACC_IDA_OFF_REQ = 3013, // 客户端关闭IDA功能请求
  HMI_TAG_PAD_CACC_IDA_OFF_RESP = 3014, // 客户端关闭IDA功能响应
  HMI_TAG_PAD_CACC_MODE_REQ = 3015, // 客户端请求设置CACC运动模式
  HMI_TAG_PAD_CACC_MODE_RESP = 3016, // 客户端设置CACC运动模式响应
  HMI_TAG_PAD_CAEB_COLLISION_SENS_REQ = 3017, // CAEB前碰撞预警灵敏度设置请求
  HMI_TAG_PAD_CAEB_COLLISION_SENS_RESP = 3018, // CAEB前碰撞预警灵敏度设置响应
  HMI_TAG_PAD_CAEB_COLLISION_WARN_ON_REQ = 3019, // 客户端请求开启CAEB前碰撞预警功能
  HMI_TAG_PAD_CAEB_COLLISION_WARN_ON_RESP = 3020, // 客户端开启CAEB前碰撞预警功能响应
  HMI_TAG_PAD_CAEB_COLLISION_WARN_OFF_REQ = 3021, // 客户端请求关闭CAEB前碰撞预警功能
  HMI_TAG_PAD_CAEB_COLLISION_WARN_OFF_RESP = 3022, // 客户端关闭CAEB前碰撞预警功能响应
  HMI_TAG_PAD_CAEB_AEB_ON_REQ = 3023, // 客户端请求开启CAEB自动紧急制动功能
  HMI_TAG_PAD_CAEB_AEB_ON_RESP = 3024, // 客户端开启CAEB自动紧急制动功能响应
  HMI_TAG_PAD_CAEB_AEB_OFF_REQ = 3025, // 客户端请求关闭CAEB自动紧急制动功能
  HMI_TAG_PAD_CAEB_AEB_OFF_RESP = 3026, // 客户端关闭CAEB自动紧急制动功能响应
  HMI_TAG_PAD_KEEP_ALIVE_REQ  = 4001, // 上位机向服务端发送探测包
  HMI_TAG_PAD_KEEP_ALIVE_RESP = 4002, // 服务端对上位机探测包的响应
};


struct PadAddressingRequestFrame {
  int tag;
  std::string destinationIp;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadAddressingRequestFrame, tag, destinationIp)
};

struct PadAddressingResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  struct data {
    std::string gSentryIp;
    int gSentryPort;
    std::string deviceSerialNum;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, gSentryIp, gSentryPort, deviceSerialNum)
  }data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadAddressingResponseFrame, tag, rsp, detail, data)
};

struct PadConnectRequestFrame {
  int tag;
  struct data {
    std::string clientIp;
    int clientPort;
    std::string deviceSerialNum;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, clientIp, clientPort, deviceSerialNum)
  }data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadConnectRequestFrame, tag, data)
};

struct PadConnectResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  struct data {
    std::string deviceSerialNum;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, deviceSerialNum)
  }data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadConnectResponseFrame, tag, rsp, detail, data)
};

struct PadKeepAliveRequestFrame {
  int tag;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadKeepAliveRequestFrame, tag)
};

struct PadKeepAliveResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  struct data {
    std::string deviceNum;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, deviceNum)
  }data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadKeepAliveResponseFrame, tag, rsp, detail, data)
};

struct PadCACCStatusFrame {
    int tag;
    struct data {
        int ACCStatus;
        int ACCfaultDetail;
        int IDAStatus;
        int IDAfaultDetail;
        std::string reason;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, ACCStatus, ACCfaultDetail, IDAStatus, IDAfaultDetail, reason)
    }data;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCStatusFrame, tag, data)
};

struct PadCAEBStatusFrame {
	int tag;
	struct data {
		int CAEBStatus;
		std::string faultDetail;
		NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, CAEBStatus, faultDetail)
	}data;
	NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCAEBStatusFrame, tag, data)
};


struct PadSystemErrorFrame {
  int type;
  std::string detail;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadSystemErrorFrame, type, detail)
};
struct PadSystemErrorVectorFrame{
  int tag;
	struct data {
    std::vector<PadSystemErrorFrame> systemError;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, systemError);
	}data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadSystemErrorVectorFrame, tag, data);
};

struct RemoteVehicleInfo {
  std::string vehicleId;
  double speed;
  double acceleration;
  double latitude;
  double longitude;
	double heading;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(RemoteVehicleInfo, vehicleId, speed, acceleration, latitude, longitude, heading);
};
struct PadV2XEgoVehInfoFrame {
	int tag;
	struct data {
    std::vector<RemoteVehicleInfo> remoteVehicleInfo;
		NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, remoteVehicleInfo);
	}data;
	NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadV2XEgoVehInfoFrame, tag, data)
};

struct PadEgoVehInfoFrame {
	int tag;
	struct data {
        double speed;
        double acceleration;
        double latitude;
        double longitude;
	      double heading;
		NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, speed, acceleration, latitude, longitude, heading)
	}data;
	NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadEgoVehInfoFrame, tag, data)
};

struct PadEgoVehCIPVInfoFrame {
	int tag;
	struct data {
        double speed;
        double acceleration;
        double latitude;
        double longitude;
	      double heading;
        double objSpeed;
        double objDistance;
        double relativeVeloc;
        double followDistance;
		NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, speed, acceleration, latitude, longitude, heading, objSpeed, objDistance, relativeVeloc, followDistance)
	}data;
	NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadEgoVehCIPVInfoFrame, tag, data)
};

struct PadCACCDecisionFrame {
	int tag;
	struct data {
		std::string CACCDecision;
		NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, CACCDecision)
	}data;
	NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCDecisionFrame, tag, data)
};

struct PadCAEBDecisionFrame {
	int tag;
	struct data {
		int CAEB;
		int preWarning;
		NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, CAEB, preWarning)
	}data;
	NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCAEBDecisionFrame, tag, data)
};

struct PadSwitchSettingsFrame {
	int tag;
	struct data {
    int CACCSwitch;
    int IDASwitch;
    int currentTimeHeadway;
    int CAEBWarningSwitch;
    int currentCollisionSensitivity;
    int CAEBSwitch;
		NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, CACCSwitch, IDASwitch, currentTimeHeadway, CAEBWarningSwitch, currentCollisionSensitivity, CAEBSwitch)
	}data;
	NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadSwitchSettingsFrame, tag, data)
};

struct PadCACCOnRequestFrame {
  int tag;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCOnRequestFrame, tag)
};

struct PadCACCOnResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCOnResponseFrame, tag, rsp, detail)
};

struct PadCACCOffRequestFrame {
  int tag;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCOffRequestFrame, tag)
};

struct PadCACCOffResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCOffResponseFrame, tag, rsp, detail)
};

struct PadCACCResetRequestFrame {
  int tag;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCResetRequestFrame, tag)
};

struct PadCACCResetResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCResetResponseFrame, tag, rsp, detail)
};

struct PadCACCCruiseControlRequestFrame {
  int tag;
	struct data {
		int setSpeed;
		NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, setSpeed)
	}data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCCruiseControlRequestFrame, tag, data)
};

struct PadCACCCruiseControlResponseFrame {
  int tag;
  int rsp;
  std::string detail;
	struct data {
		int currentSpeed;
		NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, currentSpeed)
	}data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCCruiseControlResponseFrame, tag, rsp, detail, data)
};

struct PadCACCTimeRequestFrame {
  int tag;
	struct data {
		int timeHeadway;
		NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, timeHeadway)
	}data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCTimeRequestFrame, tag, data)
};

struct PadCACCTimeResponseFrame {
  int tag;
  int rsp;
  std::string detail;
	struct data {
		int currentTimeHeadway;
		NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, currentTimeHeadway)
	}data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCTimeResponseFrame, tag, rsp, detail, data)
};

struct PadIDAOnRequestFrame {
  int tag;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadIDAOnRequestFrame, tag)
};

struct PadIDAOnResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadIDAOnResponseFrame, tag, rsp, detail)
};

struct PadIDAOffRequestFrame {
  int tag;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadIDAOffRequestFrame, tag)
};

struct PadIDAOffResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadIDAOffResponseFrame, tag, rsp, detail)
};

struct PadCACCModeRequestFrame {
	int tag;
	struct data {
		int sportMode;
		NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, sportMode)
	}data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCModeRequestFrame, tag)
};

struct PadCACCModeResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  struct data {
	  int currentSportMode;
	  NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, currentSportMode)
  }data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCACCModeResponseFrame, tag, rsp, detail)
};

struct PadCollisionSensRequestFrame {
	int tag;
	struct data {
		int collisionSensitivity;
		NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, collisionSensitivity)
	}data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCollisionSensRequestFrame, tag)
};

struct PadCollisionSensResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  struct data {
	  int currentCollisionSensitivity;
	  NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, currentCollisionSensitivity)
  }data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCollisionSensResponseFrame, tag, rsp, detail)
};

struct PadCollisionWarnOnRequestFrame {
  int tag;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCollisionWarnOnRequestFrame, tag)
};

struct PadCollisionWarnOnResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCollisionWarnOnResponseFrame, tag, rsp, detail)
};

struct PadCollisionWarnOffRequestFrame {
  int tag;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCollisionWarnOffRequestFrame, tag)
};

struct PadCollisionWarnOffResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadCollisionWarnOffResponseFrame, tag, rsp, detail)
};

struct PadAEBOnRequestFrame {
  int tag;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadAEBOnRequestFrame, tag)
};

struct PadAEBOnResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadAEBOnResponseFrame, tag, rsp, detail)
};

struct PadAEBOffRequestFrame {
  int tag;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadAEBOffRequestFrame, tag)
};

struct PadAEBOffResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(PadAEBOffResponseFrame, tag, rsp, detail)
};


#endif