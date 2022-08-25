#ifndef BE79CBC3_9771_45AD_9C4A_079955F96578
#define BE79CBC3_9771_45AD_9C4A_079955F96578

#include <cstring>
#include <iostream>

enum class MsgType : uint32_t
{
    V2X,
    CAMERA,
    CAN,
    CDS,
    HMI_DUMMY,
    HMI_PAD,
    HMI_GSENTRY,
    IPC_DUMMY,
    /* IPC RX */
	IPC_GNSS_ACC, 
	IPC_GNSS_GYRO, 
    IPC_GNSS_HEADING_PITCH_ROLL,
	IPC_GNSS_HEIGHT_TIME,
	IPC_GNSS_LAT_LONG,
	IPC_GNSS_SPEED,
	IPC_GNSS_DATA_INFO,
	IPC_GNSS_STD,
	IPC_GNSS_UTC,
    IPC_CAN,
    IPC_EVH,
    IPC_HMI_INFO,
    IPC_SYS_ERROR,
    /* IPC TX */
    IPC_HMI_CTRL,
    IPC_OBJ_INFO,
    IPC_LANE_INFO,
    IPC_GSENTRY_WARN,
    IPC_DIS2ENDLANE,
    IPC_TRAFFIC_LIGHT_INFO,
    XDS_DUMMY,
    CDD_CAMERA = 33,
    INVALID
};

struct EventMessage
{
    EventMessage(const MsgType type, const char* data_, const uint16_t len):msgType(type), msglen(len), data(nullptr)
    {
        if (data_ != nullptr)
        {
            data = new uint8_t[msglen];
            memcpy(data, data_, msglen);
        }
    }

    EventMessage(const EventMessage& ref):msgType(ref.msgType), msglen(ref.msglen), data(nullptr)
    {

        // std::cout << "reference construct" << std::endl;
        if (ref.data != nullptr)
        {
            data = new uint8_t[msglen];
            memcpy(data, ref.data, msglen);
        }
    }

    EventMessage(EventMessage&& ref):msgType(ref.msgType), msglen(ref.msglen), data(ref.data)
    {
        // std::cout << "rvalue construct" << std::endl;
        ref.data = nullptr;
    }

    ~EventMessage()
    {
        delete [] data;
    }

    EventMessage operator=(const EventMessage& ref) = delete;
public:
    MsgType msgType;
    uint16_t msglen;
    uint8_t* data;
};

#endif /* BE79CBC3_9771_45AD_9C4A_079955F96578 */
