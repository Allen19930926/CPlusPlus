#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "stdio.h"
#include "ipcm.h"
#include "ipc_data.h"
#include "event_queue.h"

#include <glog/logging.h>
extern "C" {
#include "ipc-shm.h"
}
#include <iomanip>

#define LOCAL_SHM_ADDR 0x34100000
#define IPC_SHM_SIZE 0x100000 /* 1M local shm, 1M remote shm */
#define REMOTE_SHM_ADDR (LOCAL_SHM_ADDR + IPC_SHM_SIZE)
#define INTER_CORE_TX_IRQ 2u
#define INTER_CORE_RX_IRQ 1u

extern "C"
{

static void IpcM_RxCallback(void *cb_arg, int chan_id, void *buf, size_t size);

/* data buffer configguration*/
struct ipc_shm_pool_cfg buf_pools[] = {
	{
		.num_bufs = 32,
		.buf_size = S_BUF_LEN
	},
	{
		.num_bufs = 8,
		.buf_size = M_BUF_LEN
	},
	{
		.num_bufs = 6,
		.buf_size = L_BUF_LEN
	},
};

/* data channel configuration */
struct ipc_shm_channel_cfg data_chan_cfg = {
	.type = IPC_SHM_MANAGED,
	.ch = {
		.managed = {
			.num_pools = ARRAY_SIZE(buf_pools),
			.pools = buf_pools,
			.rx_cb = IpcM_RxCallback,
			.cb_arg = NULL,
		},
	}
};

/* ipc shm configuration */
struct ipc_shm_cfg shm_cfg = {
	.local_shm_addr = LOCAL_SHM_ADDR,
	.remote_shm_addr = REMOTE_SHM_ADDR,
	.shm_size = IPC_SHM_SIZE,
	.inter_core_tx_irq = INTER_CORE_TX_IRQ,
	.inter_core_rx_irq = INTER_CORE_RX_IRQ,
	.local_core = {
		.type = IPC_CORE_DEFAULT,
		.index = IPC_CORE_INDEX_0,  /* automatically assigned */
		.trusted = IPC_CORE_INDEX_0 | IPC_CORE_INDEX_1
			   | IPC_CORE_INDEX_2 | IPC_CORE_INDEX_3
	},
	.remote_core = {
		.type = IPC_CORE_DEFAULT,
		.index = IPC_CORE_INDEX_0,  /* automatically assigned */
	},
	.num_channels = 1,
	.channels = &data_chan_cfg
};

/*************************************************************
2.1 StateMachine Variables
*************************************************************/
static IpcM_States_T IpcM_MainState = IPCM_INIT;
static uint8 IpcM_SyncCounter = 0;
static uint8 IpcM_ErrorCounter = 0;
// static uint8 IpcM_NoAliveFrameCounter = 0;
// static uint8 IpcM_LostWaitCounter = 0;
static uint8 IpcM_VersionMismatch = TRUE;
static uint8 IpcM_Inited = FALSE;
// static uint8 IpcM_SyncRecieved = FALSE;
static uint8 IpcM_Ack1Received = FALSE;
// static uint8 IpcM_Ack2Received = FALSE;
static uint8 IpcM_LastRxMessage[IPCF_SHM_MAX_BUFFER_LENGTH];
static uint32 IpcM_lastErrorCode;


static IpcM_Result_T IpcM_SendSync(void);
// static IpcM_Result_T IpcM_SendAck1(IpcM_Result_T versionCheckResult);
static IpcM_Result_T IpcM_SendAck2(void);
static IpcM_Result_T IpcM_SendKeepAliveFrame(uint8 temp, uint8 cpuLoad, uint32 timestamp);
// static void IpcM_ErrorCallout(void);
static void IpcM_ProtocolParse(uint32);
// static void IpcM_CheckKeepAliveFrame(void);
// static void IpcM_VisionStreamHandler();

static void IpcM_HandleError(uint32 error)
{
	LOG(INFO) << "IPCM Error: " << error;
	IpcM_lastErrorCode = error;
}

static inline void IpcM_Memcpy(const void* target, const void* src, int len)
{
	int i;
    for (i = 0; i < len; i++)
    {
    	*((uint8 *)target + i) = ((uint8 *)src)[i];
    }
}

static void IpcM_Appl_rxCallback(IpcM_FrameTypes_T frameType, const char * data, const uint32 len)
{
	if (frameType == IPC_APPL_FRAME_TIME_SYNC)
	{
		if (sizeof(timespec) != len)
		{
			LOG(ERROR) << ("received data size does not match sizeof timespec");
			return;
		}
		LOG(INFO) << "receive time sync frame";
		
		struct timespec ts;
    	IpcM_Memcpy(&ts, data, sizeof(ts));
		clock_settime(CLOCK_REALTIME, &ts);
	}
    else if (frameType == IPC_APPL_FRAME_GNSS_DATA) {
		if (len != sizeof(IPC_GNSS_Data))
		{
			LOG(ERROR) << ("received data size does not match sizeof IPC_GNSS_Data");
			return;
		}
		CDD_FUSION_EVENT_QUEUE.push({MsgType::IPC_GNSS_DATA, data , static_cast<uint16_t>(len)});
    }
	else if (frameType == IPC_APPL_FRAME_GNSS_UTC) {
		if (len != sizeof(IPC_GNSS_UTC))
		{
			LOG(ERROR) << ("received data size does not match sizeof IPC_GNSS_UTC");
			return;
		}
		CDD_FUSION_EVENT_QUEUE.push({ MsgType::IPC_GNSS_UTC, data , static_cast<uint16_t>(len) });
	}
	else if (frameType == IPC_APPL_FRAME_HMI_INFO) {
		if (len != sizeof(FuncCoord_FAM_HMI_Info))
		{
			LOG(ERROR) << ("received data size does not match sizeof FuncCoord_FAM_HMI_Info");
			return;
		}
		CDD_FUSION_EVENT_QUEUE.push({ MsgType::IPC_HMI_INFO, data , static_cast<uint16_t>(len) });
	}
	else if (frameType == IPC_APPL_FRAME_EVH_SUBJECT_INFO) {
		if (len != sizeof(EVH_SubjectInfo_BUS))
		{
			LOG(ERROR) << ("received data size does not match sizeof EVH_SubjectInfo_BUS");
			return;
		}
		CDD_FUSION_EVENT_QUEUE.push({ MsgType::IPC_EVH, data , static_cast<uint16_t>(len) });
	}
	else if (frameType == IPC_APPL_FRAME_CAN_DATA) {
		if (len != sizeof(IPC_CAN_Data))
		{
			LOG(ERROR) << ("received data size does not match sizeof CDS_CAN_Data");
			return;
		}
		CDD_FUSION_EVENT_QUEUE.push({ MsgType::IPC_CAN, data , static_cast<uint16_t>(len) });
	}
	else if (frameType == IPC_APPL_FRAME_SYS_ERROR) {
		if (len != sizeof(IPC_System_Error))
		{
			LOG(ERROR) << ("received data size does not match sizeof IPC_System_Error");
			return;
		}
		CDD_FUSION_EVENT_QUEUE.push({ MsgType::IPC_SYS_ERROR, data , static_cast<uint16_t>(len) });
	}
	else {
		LOG(ERROR) << ("unexpected message, frameType: ") << frameType;
		return;
	}
}

void IpcM_Init(void)
{
	int err;
	err = ipc_shm_init(&shm_cfg);
	if (err == 0u)
	{
		DLOG(INFO) << "init succeed, goto idle state";
	}
	else
	{
		LOG(ERROR) << "init failed, error code: " << err;
		return;
	}
	IpcM_Inited = TRUE;
}

void IpcM_MainFunction(void)
{
	static int counter = 0;
    static boolean sync_once = FALSE;
	static IpcM_States_T old_state = IPCM_INIT;
	IpcM_Result_T ipcSendResult = IPCM_SEND_ERROR;
	int err = 0;
	if (old_state != IpcM_MainState) {
		DLOG(INFO) << "state transit from: " << old_state << " to " << IpcM_MainState;
		old_state = IpcM_MainState;
	}
	if (counter++ == 20) {
		counter = 0;
		DLOG(INFO) << "IpcM_MainState: " << IpcM_MainState;
	}
	switch (IpcM_MainState)
	{
        case IPCM_INIT:
            if (IpcM_Inited)
            {
				IpcM_MainState = IPCM_IDLE;
            }
            break;       
		case IPCM_IDLE:
            if (!sync_once)
            {
                ipcSendResult = IpcM_SendSync();
                if(ipcSendResult == IPCM_OK)
                {
                    DLOG(INFO) << "goto sync state";
                    IpcM_MainState = IPCM_SYNC;
                    IpcM_SyncCounter++;
                    IpcM_ErrorCounter = 0;
                }
                else
                {
                    LOG(ERROR) << "init failed, error code: " << err;
                }    
            }
            sync_once = TRUE;
			break;
		case IPCM_SYNC:
			if ((IpcM_Ack1Received == TRUE) && (IpcM_VersionMismatch == FALSE))
			{
				ipcSendResult = IpcM_SendAck2();
				if(ipcSendResult == IPCM_OK)
				{
                    DLOG(INFO) << "goto running state";
					IpcM_MainState = IPCM_RUNNING;
					IpcM_SyncCounter = 0;
					IpcM_ErrorCounter = 0;
				}
				else
				{
                    LOG(ERROR) << "send ACK2 failed, error code: " << err;
				}
			}
			else if ((IpcM_Ack1Received == TRUE) && (IpcM_VersionMismatch == TRUE))
			{
				IpcM_MainState = IPCM_LOST;
                LOG(ERROR) << "IPC verion mismatch";
			}
			else
			{
                //waiting for ack1
                IpcM_SyncCounter++;
                IpcM_ErrorCounter = 0;
                if(IpcM_SyncCounter > IPCM_SYNC_TIMEOUT_MS)
                {
					DLOG(INFO) << "sync timeout, goto lost state";
                    IpcM_MainState = IPCM_LOST;
                }
			}
			break;
		case IPCM_RUNNING:
			//check keep-alive-frame from A-core, the vision data stream is handled in IpcM_RxCallback.
			IpcM_SendKeepAliveFrame(0, 0, 0);
			break;
		case IPCM_LOST:
			break;
        default:
            break;
		
	}
}

void IpcM_Free()
{
	if (IpcM_Inited) {
		ipc_shm_free();
		IpcM_Inited = FALSE;
	}
}

//common IPC send API
IpcM_Result_T IpcM_Send(const IpcM_FrameTypes_T frameType, const void* srcAddress, const uint16 msgLen)
{
	uint32 err = 0;
    uint8 chanId = IPC_CHANNEL_0;
	uint8 *buf = NULL;

	if (!IpcM_Inited)
	{
        LOG(ERROR) << "IPCM is not initialized";
		return IPCM_INIT_FAILED;
	}
	

	buf = (uint8 *)ipc_shm_acquire_buf(chanId, msgLen + 4); //add by Tao, give 0 to instance no.
	if (!buf) {
        LOG(ERROR) << "IPCM require buffer error";
		return IPCM_BUFFER_ERROR;
	}
	// LOG(INFO) << "acquired buffer on: " << std::hex << reinterpret_cast<std::uintptr_t>(buf);
    *((uint32 *)buf) = (uint32)frameType;


	/* write data to acquired buffer */
//    for (i = 0; i < msgLen; i++)
//    {
//    	*(buf + 4 + i) = ((uint8 *)srcAddress)[i];
//    }
    /* might corrupt when use memcpy */
    IpcM_Memcpy(buf + 4, srcAddress, msgLen);

	// for (int i = 0; i < msgLen + 4; i++)
	// 	printf("%02x", *(buf + i));
	// printf("\n");
    // 	std::cout << std::hex << *(buf + i) << " ";
	// std::cout << std::endl;

	/* send data to remote peer */
	err = ipc_shm_tx(chanId, buf, msgLen + 4); //add by Tao, give 0 to instance no.
	if (err) {
        LOG(ERROR) << "IPCM send error: " << std::hex << err;
		return IPCM_SEND_ERROR;
	}
	return IPCM_OK;
}
/*
 * data channel Rx callback: increment Rx counter, release buffer and signal
 * demo task a ping message has been received.
 */
static void IpcM_RxCallback(void *arg, int chanId, void *buf, size_t size)
{
	int err;

	if (size > IPCF_SHM_MAX_BUFFER_LENGTH) {
		IpcM_HandleError(IPCM_INVALID_ARG);
		/* release the buffer */
		// err = ipc_shm_release_buf(chanId, buf);
		// // LOG(INFO) << "release buffer on: " << std::hex << reinterpret_cast<std::uintptr_t>(buf);
		// if (err) {
		// 	LOG(ERROR) << "failed to release buf, error code: " << err;
		// }
		// return;
	}

	/* consume received data */
	IpcM_Memcpy(IpcM_LastRxMessage, buf, size);

	/* release the buffer */
	err = ipc_shm_release_buf(chanId, buf);
	// LOG(INFO) << "release buffer on: " << std::hex << reinterpret_cast<std::uintptr_t>(buf);
	if (err) {
        LOG(ERROR) << "failed to release buf, error code: " << err;
		// return;
	}
	
	/* protocol parse, add by Tao*/
	IpcM_ProtocolParse(size);
}

static void IpcM_ProtocolParse(uint32 size)
{
	uint32 frameType;
	IpcM_ACK1_Frame_T ack1Frame;
	IpcM_Memcpy(&frameType, IpcM_LastRxMessage, 4);
    // LOG(INFO) << ("received frame: %d, state: %d \n", frameType, IpcM_MainState);
	if (frameType == IPCM_FRAME_DEBUG)
	{
		LOG(INFO) << ((const char *)(IpcM_LastRxMessage + 4));
		return;
	}
	switch (IpcM_MainState)
	{
		case IPCM_IDLE:
			break;
		case IPCM_SYNC:
			if (frameType == IPCM_FRAME_ACK1)
            {
				IpcM_Memcpy(&ack1Frame, IpcM_LastRxMessage + 4, sizeof(ack1Frame));
				if ((ack1Frame.VersionCheckResult == IPCM_OK))
				{
					IpcM_VersionMismatch = FALSE;
				}
				IpcM_Ack1Received = TRUE;
            }
			break;
		case IPCM_RUNNING:
		    if (frameType > IPCM_FRAME_DEBUG)
			{
				IpcM_Appl_rxCallback((IpcM_FrameTypes_T)frameType, (const char *)IpcM_LastRxMessage + 4, size - 4);
			}
			
			break;
		case IPCM_LOST:
			//do nothing
			break;
		default:
			//do nothing
			break;
	}
}

//send out sync msg from M-core
static IpcM_Result_T IpcM_SendSync(void)
{
	IpcM_SYNC_Frame_T syncFrame;
	IpcM_Result_T ret_value;

	syncFrame.ProtocolVersion = IPCM_PROTOCOL_VERSION;
	syncFrame.DataVersion     = IPCM_DATA_VERSION;

	ret_value = IpcM_Send(IPCM_FRAME_SYNC, &syncFrame, sizeof(syncFrame));
	return ret_value;
}

//send out ack2 msg from M-core
static IpcM_Result_T IpcM_SendAck2(void)
{
	IpcM_ACK2_Frame_T ack2Frame;
	IpcM_Result_T retValue;
	retValue = IpcM_Send(IPCM_FRAME_ACK2, &ack2Frame, sizeof(ack2Frame));
	return retValue;
}

// static void IpcM_ErrorCallout(void)
// {
// 	//add coding to report to DET
// 	IpcM_ErrorCounter++;
// 	if(IpcM_ErrorCounter > IPC_ERROR_LIMIT_10S)
// 	{
// 		IpcM_MainState = IPCM_LOST;
// 	}
// }

// //send out keep-alive-frame from A-core
static IpcM_Result_T IpcM_SendKeepAliveFrame(uint8 temp, uint8 cpuLoad, uint32 timestamp)
{
	IpcM_KeepAlive_Frame_T keepAliveFrame;
	IpcM_Result_T retValue;
	keepAliveFrame.aCoreTemp = temp;
	keepAliveFrame.aCoreCPULoad = cpuLoad;
	keepAliveFrame.timeStamp = timestamp;

	retValue = IpcM_Send(IPCM_FRAME_KEEPALIVE, &keepAliveFrame, sizeof(keepAliveFrame));
	return retValue;
}

}


void IpcM::Init() {
  IpcM_Init();
}

void IpcM::DeInit() {
	// DLOG(INFO) << "stopping IpcM thread";
	LOG(INFO) << "ipcm free resource";
    IpcM_Free();
}

void IpcM::Run() {
	IpcM_MainFunction();
}

int IpcM::Write(const IpcM_FrameTypes_T frameType, const void* srcAddress, const uint16 msgLen) {
	if (IpcM_MainState != IPCM_RUNNING) {
		return -1;
	}
	LOG(INFO) << "ipcm send frame, type: " << (int)frameType << ", size:" << msgLen;
    IpcM_Send(frameType, srcAddress, msgLen);
    return 0;
}
