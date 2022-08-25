#pragma once

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// #include <glog/logging.h>

#define IPCM_PROTOCOL_VERSION 01
#define IPCM_DATA_VERSION 01

#define S_BUF_LEN 16
#define M_BUF_LEN 1024
#define L_BUF_LEN 4096*4
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define IPC_CHANNEL_0 0
#define IPCF_INSTANCE  0u
#define IPCF_SHM_MAX_BUFFER_LENGTH	4096

#define IPCM_SYNC_TIMEOUT_MS 20000
#define IPC_ERROR_LIMIT_10S 20000
#define IPCM_ALIVE_TIMEOUT 20000
#define IPC_LOST_WAIT_10s 20000

#ifndef AUTOSAR_TYPES
#define AUTOSAR_TYPES
//add AUTOSAR typedef to avoid linux compile error
typedef unsigned char uint8;
typedef unsigned short int uint16;
typedef unsigned int uint32;
typedef uint8 uint8_t;
typedef uint16 uint16_t;
typedef uint32 uint32_t;
typedef uint8 boolean;
#define TRUE 1u
#define FALSE 0u
#endif

typedef enum
{
	IPCM_FRAME_INVALID = 0,
	IPCM_FRAME_SYNC,
	IPCM_FRAME_ACK1,
	IPCM_FRAME_ACK2,
	IPCM_FRAME_KEEPALIVE,
	IPCM_FRAME_DEBUG,
	IPC_APPL_FRAME_TIME_SYNC,
	IPC_APPL_FRAME_GNSS_ACC,
	IPC_APPL_FRAME_GNSS_GYRO,
	IPC_APPL_FRAME_GNSS_HEADING_PITCH_ROLL,
	IPC_APPL_FRAME_GNSS_HEIGHT_TIME,
	IPC_APPL_FRAME_GNSS_LAT_LONG,
	IPC_APPL_FRAME_GNSS_SPEED,
	IPC_APPL_FRAME_GNSS_DATA_INFO,
	IPC_APPL_FRAME_GNSS_STD,
	IPC_APPL_FRAME_GNSS_UTC,
	IPC_APPL_FRAME_CORE_A_STATUS,
	IPC_APPL_FRAME_FUSION_CAM_OBJ,
	IPC_APPL_FRAME_FUSION_GSENTRY_OBJ,
	IPC_APPL_FRAME_FUSION_LANE_INFO,
	IPC_APPL_FRAME_DIST_TO_END_LANE,
	IPC_APPL_FRAME_TRAFFIC_LIGHT_INFO,
	IPC_APPL_FRAME_GSENTRY_WARN_INFO,
	IPC_APPL_FRAME_HMI_CTRL,
	IPC_APPL_FRAME_HMI_INFO,                     /* M -> A */
    IPC_APPL_FRAME_EVH_SUBJECT_INFO,             /* M -> A */
    IPC_APPL_FRAME_CAN_DATA,                     /* M -> A */
    IPC_APPL_FRAME_SYS_ERROR,                    /* M -> A */
} IpcM_FrameTypes_T;

typedef enum{
	IPCM_OK = 0,
	IPCM_INIT_FAILED,
	IPCM_IDLE_TIMEOUT,
	IPCM_SYNC_TIMEOUT,
	IPCM_IDLE_VER_MISMATCH,
	IPCM_RUN_TIMEOUT,
	IPCM_BUFFER_ERROR,
	IPCM_INVALID_ARG,
	IPCM_SEND_WRONG_STATE,
	IPCM_SEND_ERROR
} IpcM_Result_T;

typedef enum{
	IPCM_INIT = 0,
	IPCM_IDLE,
	IPCM_SYNC,
	IPCM_RUNNING,
	IPCM_LOST
} IpcM_States_T;

typedef enum{
	IPCM_ERROR_LIMIT = 0,
	IPCM_SYNC_LIMIT,
	IPCM_DATA_ABSENT
} IpcM_LostReason_T;

#pragma pack(push, 4)

typedef struct
{
	uint8 ProtocolVersion;
	uint8 DataVersion;
	uint8 Reserved[2];
} IpcM_SYNC_Frame_T;

typedef struct
{
	uint8 ProtocolVersion;
	uint8 DataVersion;
	uint8 VersionCheckResult;
	uint8 Reserved;
} IpcM_ACK1_Frame_T;

typedef struct
{
	uint8 Reserved[4];
} IpcM_ACK2_Frame_T;

typedef struct
{
	uint8 aCoreTemp;
	uint8 aCoreCPULoad;
	uint8 Reserved[2];
	uint32 timeStamp;
} IpcM_KeepAlive_Frame_T;

#pragma pack(pop)

	
class IpcM {
 public:

  IpcM() {}

  ~IpcM() {
	// DLOG(INFO) << "stopping IpcM thread";
    start_thread_ = false;
    if (ipcm_thread_.joinable()) {
      ipcm_thread_.join();
    }
  }

  void Run();

  static int Write(const IpcM_FrameTypes_T frameType, const void* srcAddress, const uint16 msgLen);

 private:
  std::thread ipcm_thread_;
  std::atomic_bool start_thread_{false};
};


