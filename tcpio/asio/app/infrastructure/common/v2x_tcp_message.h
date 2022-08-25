#ifndef D9065D7E_95F5_477F_908E_EA22EB175C6C
#define D9065D7E_95F5_477F_908E_EA22EB175C6C

#include <string>
#include <cstring>
#include <iostream>
#include "v2x_data_struct.h"

class V2xTcpMessage {
public:
    static constexpr std::size_t HeaderLength = sizeof(V2X::V2xAdasMsgHeader);
    static constexpr std::size_t MaxBodyLength = 10 * 1024;

    V2xTcpMessage() : bodyLength(0), packNum(0), packLen(0) {}
    V2xTcpMessage(const std::string msg)
    {
        packNum = 0;
        packLen = 0;
        bodyLength = msg.length() + 1;
        memcpy(dataBuffer, msg.data(), msg.length() + 1);
    }

    V2xTcpMessage(const char* buf , const uint16_t len)
    {
        packLen = 0;
        packNum = 0;
        if (len < HeaderLength + MaxBodyLength)
        {
            memcpy(dataBuffer, buf, len);
        }
    }

    char *Data() { return dataBuffer; }

    std::size_t Length() const { return HeaderLength + bodyLength; }

    char *Body() { return dataBuffer + HeaderLength; }

    const int BodyLength() { return bodyLength; }

    bool DecodeHeader()
    {
        V2X::V2xAdasMsgHeader head = *reinterpret_cast<V2X::V2xAdasMsgHeader*>(&dataBuffer[0]); 
        bodyLength = head.msgLen;
        packNum++;
        packLen += bodyLength + HeaderLength;
        // std::cout << "msgid = " << head.msgId  << "msglen = " << head.msgLen  << "recieve num = " << packNum  << "total length = " << packLen << std::endl;
        return true;
    }

protected:
    char dataBuffer[HeaderLength + MaxBodyLength];
    std::size_t bodyLength;
    uint32_t packNum;
    uint32_t packLen;
};

#endif /* D9065D7E_95F5_477F_908E_EA22EB175C6C */
