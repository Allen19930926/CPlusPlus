#ifndef CDSPROXY_TCP_MESSAGE_H_
#define CDSPROXY_TCP_MESSAGE_H_

#include "default_chat_message.h"
#include <functional>

class CdsTcpMessage : public DefaultChatMessage {
public:
    static constexpr std::size_t HeaderLength = 0;

    CdsTcpMessage() : DefaultChatMessage() {}
    CdsTcpMessage(const char* buf , const uint16_t len)
    {
        memcpy(dataBuffer, buf, len);
        bodyLength = len;
        // std::cout<<"len = "<<len<<std::endl;
    }

    bool DecodeHeader()
    {
        // throw std::bad_function_call();
        bodyLength = 50;
        return true;
    }
};


#endif