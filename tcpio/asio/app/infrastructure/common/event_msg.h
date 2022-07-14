#ifndef BE79CBC3_9771_45AD_9C4A_079955F96578
#define BE79CBC3_9771_45AD_9C4A_079955F96578

#include <cstring>

enum class MsgType : uint32_t
{
    V2X,
    CAMERA,
    CAN,
    HMI,
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
        if (ref.data != nullptr)
        {
            data = new uint8_t[msglen];
            memcpy(data, ref.data, msglen);
        }
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
