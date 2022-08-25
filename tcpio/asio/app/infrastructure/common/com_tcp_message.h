#ifndef COM_TCP_MESSAGE_H
#define COM_TCP_MESSAGE_H

#include <cstddef>
#include <string>
#include <cstring>
#include "event_msg.h"

class ComTcpMessage {
public:
  static constexpr std::size_t HeaderLength = 12;
  static constexpr std::size_t MaxBodyLength = 1024 * 20;
  MsgType FrameType;

  ComTcpMessage() : bodyLength(0) {}

  ComTcpMessage(EventMessage msg): 
      ComTcpMessage(msg.msgType, msg.data, msg.msglen) {
  }

  ComTcpMessage(MsgType frameType, const std::string & data): 
      ComTcpMessage(frameType, data.c_str(), data.length()) {
  }

  ComTcpMessage(MsgType frameType, const void * data, const size_t len) {
    bodyLength = len;
    dataBuffer[0] = 0;
    dataBuffer[1] = 0xFF;
    dataBuffer[2] = 0;
    dataBuffer[3] = 0;
    dataBuffer[4] = static_cast<unsigned char>(bodyLength >> 24);
    dataBuffer[5] = static_cast<unsigned char>(bodyLength >> 16);
    dataBuffer[6] = static_cast<unsigned char>(bodyLength >> 8);
    dataBuffer[7] = static_cast<unsigned char>(bodyLength);
    auto ft = static_cast<int>(frameType);
    dataBuffer[8] = static_cast<unsigned char>(ft >> 24);
    dataBuffer[9] = static_cast<unsigned char>(ft >> 16);
    dataBuffer[10] = static_cast<unsigned char>(ft >> 8);
    dataBuffer[11] = static_cast<unsigned char>(ft);
    FrameType = frameType;
    std::memcpy(dataBuffer + HeaderLength, data, len);

  }

  const char *Data() const { return dataBuffer; }
  char *Data() { return dataBuffer; }

  std::size_t Length() const { return HeaderLength + bodyLength; }

  const char *Body() const { return dataBuffer + HeaderLength; }
  char *Body() { return dataBuffer + HeaderLength; }

  const int BodyLength() { return bodyLength; }

  bool DecodeHeader() {
    if (static_cast<unsigned char>(dataBuffer[0]) == 0 &&
        static_cast<unsigned char>(dataBuffer[1]) == 0xFF &&
        static_cast<unsigned char>(dataBuffer[2]) == 0 &&
        static_cast<unsigned char>(dataBuffer[3]) == 0) {
      bodyLength =
          static_cast<int>(static_cast<unsigned char>(dataBuffer[4]) << 24 |
                           static_cast<unsigned char>(dataBuffer[5]) << 16 |
                           static_cast<unsigned char>(dataBuffer[6]) << 8 |
                           static_cast<unsigned char>(dataBuffer[7]));
      FrameType = MsgType(
          static_cast<int>(static_cast<unsigned char>(dataBuffer[8]) << 24 |
                           static_cast<unsigned char>(dataBuffer[9]) << 16 |
                           static_cast<unsigned char>(dataBuffer[10]) << 8 |
                           static_cast<unsigned char>(dataBuffer[11])));
      if (bodyLength > MaxBodyLength) {
        bodyLength = 0;
        return false;
      }
      return true;
    } else {
      return false;
    }
  }

private:
  char dataBuffer[HeaderLength + MaxBodyLength];
  std::size_t bodyLength;
};

#endif

