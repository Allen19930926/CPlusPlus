#ifndef B524BFE0_CADF_4E3E_8768_519267D72C65
#define B524BFE0_CADF_4E3E_8768_519267D72C65


#include "nlohmann/nlohmann/json.hpp"
#include <cstddef>
#include <string>

using json = nlohmann::json;

enum HMI_TAG_DEF {
    HMI_TAG_KEEP_ALIVE_REQ = 7,
    HMI_TAG_KEEP_ALIVE_RESP = 8,
};

struct gSentryKeepAliveRequestFrame {
    int tag;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(gSentryKeepAliveRequestFrame, tag)

struct gSentryKeepAliveResponseFrame {
    int tag;
    int rsp;
    std::string detail;
    std::string deviceSerialNumber;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(gSentryKeepAliveResponseFrame, tag, rsp,
                                   detail, deviceSerialNumber)

class HmiMessage {
public:
    static constexpr std::size_t HeaderLength = 8;
    static constexpr std::size_t MaxBodyLength = 4000;

    HmiMessage() : bodyLength(0) {}

    const char *Data() const { return dataBuffer; }
    char *Data() { return dataBuffer; }

    std::size_t Length() const { return HeaderLength + bodyLength; }

    const char *Body() const { return dataBuffer + HeaderLength; }
    char *Body() { return dataBuffer + HeaderLength; }

    const int BodyLength() { return bodyLength; }

    bool DecodeHeader() {
        if (dataBuffer[0] == 0 && dataBuffer[1] == 0xFF && dataBuffer[2] == 0 &&
            dataBuffer[3] == 0) {
        int bodyLength =
            static_cast<int>(static_cast<unsigned char>(dataBuffer[4]) << 24 |
                            static_cast<unsigned char>(dataBuffer[5]) << 16 |
                            static_cast<unsigned char>(dataBuffer[6]) << 8 |
                            static_cast<unsigned char>(dataBuffer[7]));
        if (bodyLength > MaxBodyLength) {
            bodyLength = 0;
            return false;
        }
        return true;
        } else {
        return false;
        }
    }

    json ToJson() {
        return json::parse(dataBuffer + HeaderLength,
                        dataBuffer + HeaderLength + bodyLength);
    }

private:
    char dataBuffer[HeaderLength + MaxBodyLength];
    std::size_t bodyLength;
};


#endif /* B524BFE0_CADF_4E3E_8768_519267D72C65 */
