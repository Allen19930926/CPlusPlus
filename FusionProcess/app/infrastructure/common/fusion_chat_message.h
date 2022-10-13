#ifndef E3DF7E56_B5EC_46DD_B5F2_D19B8AC0EB70
#define E3DF7E56_B5EC_46DD_B5F2_D19B8AC0EB70

#include <string>
#include <cstring>

struct FusionChatHeader
{
    uint32_t sensor_type;
    uint32_t time_stamp;
    uint32_t objectNum;
    uint32_t bodyLength;
};

class FusionChatMessage {
public:
    static constexpr std::size_t HeaderLength = 16;
    static constexpr std::size_t MaxBodyLength = 1024 * 10;

    FusionChatMessage() : bodyLength(0) {}

    FusionChatMessage(const char* buf , const uint16_t len)
    {
        if (len < HeaderLength + MaxBodyLength)
        {
            memcpy(dataBuffer, buf, len);
            bodyLength = len;
        }
    }

    char *Data() { return dataBuffer; }

    std::size_t Length() const { return HeaderLength + bodyLength; }

    char *Body() { return dataBuffer + HeaderLength; }

    const int BodyLength() { return bodyLength; }

    bool DecodeHeader()
    {
        const FusionChatHeader& head = *reinterpret_cast<FusionChatHeader*>(&dataBuffer);
        if (head.bodyLength >= MaxBodyLength)
        {
            return false;
        }
        bodyLength = head.bodyLength;
        return true;
    }

protected:
    char dataBuffer[HeaderLength + MaxBodyLength];
    std::size_t bodyLength;
};

#endif /* E3DF7E56_B5EC_46DD_B5F2_D19B8AC0EB70 */
