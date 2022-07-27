#ifndef D918F069_1E0D_400D_B8C1_2024B88530C9
#define D918F069_1E0D_400D_B8C1_2024B88530C9

#include <string>

class BasciChatMessage {
public:
    static constexpr std::size_t HeaderLength = 0;
    static constexpr std::size_t MaxBodyLength = 1024;

    BasciChatMessage() : bodyLength(0) {}
    BasciChatMessage(const std::string msg)
    {
        bodyLength = msg.length() + 1;
        memcpy(dataBuffer, msg.data(), msg.length() + 1);
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

private:
    char dataBuffer[HeaderLength + MaxBodyLength];
    std::size_t bodyLength;
};

#endif /* D918F069_1E0D_400D_B8C1_2024B88530C9 */
