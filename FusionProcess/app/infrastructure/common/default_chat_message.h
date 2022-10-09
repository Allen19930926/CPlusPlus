#ifndef D918F069_1E0D_400D_B8C1_2024B88530C9
#define D918F069_1E0D_400D_B8C1_2024B88530C9

#include <string>

class DefaultChatMessage {
public:
    //static constexpr std::size_t HeaderLength = 4;
    static constexpr std::size_t HeaderLength = 0;
    static constexpr std::size_t MaxBodyLength = 1024 * 10;

    DefaultChatMessage() : bodyLength(0) {}
    DefaultChatMessage(const std::string msg)
    {
        std::size_t len = msg.length() + 1;
        if (len < HeaderLength + MaxBodyLength)
        {
            bodyLength = len;
            memcpy(dataBuffer, msg.data(), len);
        }
    }

    DefaultChatMessage(const char* buf , const uint16_t len)
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
        bodyLength = 50; 
        return true;
    }

protected:
    char dataBuffer[HeaderLength + MaxBodyLength];
    std::size_t bodyLength;
};

#endif /* D918F069_1E0D_400D_B8C1_2024B88530C9 */
