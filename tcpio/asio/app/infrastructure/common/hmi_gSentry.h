#ifndef HMI_GSENTRY_H_
#define HMI_GSENTRY_H_

#include "nlohmann/json.hpp"

using json = nlohmann::json;

enum class HMI_GSENTRY_TAG_DEF: int{

  GSENTRY_HMI_CON_RESP = 4,           // pad连接请求响应
  GSENTRY_HMI_INIT_RESP = 6,          // pad初始化请求响应
  GSENTRY_HMI_KEEP_ALIVE_REQ = 7,
  GSENTRY_HMI_KEEP_ALIVE_RESP = 8,
  GSENTRY_HMI_CLOSEV2X_RESP = 10,     // pad关闭所有V2X功能请求响应
  GSENTRY_HMI_OPENV2X_RESP = 12,      //pad开启所有V2X功能请求响应
  GSENTRY_HMI_CLOSEONEV2X_RESP = 14,  // pad关闭单项V2X功能请求响应
  GSENTRY_HMI_OPENONEV2X_RESP = 16,   // pad开启单项V2X功能请求响应
  GSENTRY_HMI_SENCE_RESP = 18,          //pad设置gentry灵敏度请求响应
  GSENTRY_HMI_GSENTRY_STATUS_REQ = 19,  //gsentry上报v2x/gsentry状态请求响应（图标显示需求）
  GSENTRY_HMI_V2X_STATUS_RESP = 22,     //pad查询v2x/gsentry状态请求响应    
  GSENTRY_HMI_V2XSWITCH_STATUS_RESP = 24,    //pad查询gV2X场景开关状态请求响应 
  GSENTRY_HMI_SENSESTATUS_RESP = 26,         //pad获取gentry灵敏度请求响应

  GSENTRY_HMI_CAEB_COLLISION_SENS_REP = 1001,   //gSentry给的v2v事件
  GSENTRY_HMI_V2IWARNING = 1002,    //gSentry上报v2I事件
  GSENTRY_HMI_RLVWINFO = 1003,      //gSentry上报RLVW事件
  GSENTRY_HMI_VRUCWINFO = 1004,     //gSentry上报VRUCW事件
  GSENTRY_HMI_TRAFFIClIGHT = 1005,  //gSentry上报Traffic Light事件
  GSENTRY_HMI_SLWINFO = 1006,       //gSentry上报SLW事件
  GSENTRY_HMI_HOSTDATA = 1007,      //gSentry上报本车数据
  GSENTRY_HMI_REMOTEDATA = 1008,    //gSentry上报远车数据
  GSENTRY_HMI_GLOSAINFO = 1009,     //gSentry上报GLOSA数据
};

struct gSentryKeepAliveRequestFrame {
  int tag;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(gSentryKeepAliveRequestFrame, tag)
};
//JSON_HELPER(gSentryKeepAliveRequestFrame, tag)

struct gSentryKeepAliveResponseFrame {
  int tag;
  int rsp;
  std::string detail;
  struct data {
    std::string deviceSerialNumber;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(data, deviceSerialNumber)
  }data;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(gSentryKeepAliveResponseFrame, tag, rsp, detail, data)
};
//JSON_HELPER(gSentryKeepAliveResponseFrame, tag, rsp, detail, deviceSerialNumber)
#endif