#ifndef XDS_TCP_MESSAGE_H
#define XDS_TCP_MESSAGE_H

#include "default_chat_message.h"
#include <functional>

class XdsTcpMessage : public DefaultChatMessage {
public:
    static constexpr std::size_t HeaderLength = 0;

    XdsTcpMessage() : DefaultChatMessage() {
        
    }

    XdsTcpMessage(const std::string msg)
    {
        bodyLength = msg.length();
        memcpy(dataBuffer, msg.data(), msg.length());
    }

    bool DecodeHeader()
    {
        // throw std::bad_function_call();
        bodyLength = 50;
        return true;
    }
};

#endif
