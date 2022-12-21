#ifndef COM_TCP_MESSAGE_H
#define COM_TCP_MESSAGE_H

#include <cstddef>
#include <string>
#include <cstring>
#include "event_msg.h"
#include <chrono>
#include <vector>

using namespace std::chrono;
using time_stamp = std::chrono::time_point<std::chrono::system_clock,
                                           std::chrono::microseconds>;

class ComTcpMessage {
public:
  static constexpr int HeaderLength = 20;
  static constexpr int MaxBodyLength = 40000;
  MsgType FrameType;

  ComTcpMessage() : bodyLength(0) {}

  ComTcpMessage(EventMessage msg): 
      ComTcpMessage(msg.msgType, msg.data, msg.msglen) {
  }

  ComTcpMessage(MsgType frameType, const std::string & data): 
      ComTcpMessage(frameType, data.c_str(), data.length()) {
  }

  ComTcpMessage(MsgType frameType, const void * data, const size_t len) {
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>
              (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    bodyLength = len;
    dataBuffer.push_back(0);
    dataBuffer.push_back(0xFF);
    dataBuffer.push_back(0);
    dataBuffer.push_back(0);
    dataBuffer.push_back(static_cast<unsigned char>(bodyLength >> 24));
    dataBuffer.push_back(static_cast<unsigned char>(bodyLength >> 16));
    dataBuffer.push_back(static_cast<unsigned char>(bodyLength >> 8));
    dataBuffer.push_back(static_cast<unsigned char>(bodyLength));
    auto ft = static_cast<int>(frameType);
    dataBuffer.push_back(static_cast<unsigned char>(ft >> 24));
    dataBuffer.push_back(static_cast<unsigned char>(ft >> 16));
    dataBuffer.push_back(static_cast<unsigned char>(ft >> 8));
    dataBuffer.push_back(static_cast<unsigned char>(ft));


    dataBuffer.push_back(static_cast<unsigned char>(nanoseconds >> 56));
    dataBuffer.push_back(static_cast<unsigned char>(nanoseconds >> 48));
    dataBuffer.push_back(static_cast<unsigned char>(nanoseconds >> 40));
    dataBuffer.push_back(static_cast<unsigned char>(nanoseconds >> 32));
    dataBuffer.push_back(static_cast<unsigned char>(nanoseconds >> 24));
    dataBuffer.push_back(static_cast<unsigned char>(nanoseconds >> 16));
    dataBuffer.push_back(static_cast<unsigned char>(nanoseconds >> 8));
    dataBuffer.push_back(static_cast<unsigned char>(nanoseconds));
    FrameType = frameType;
    bodyLength = len;

    dataBuffer.insert(dataBuffer.end(), &(static_cast<const char *>(data)[0]), &(static_cast<const char *>(data)[len]));

  }

  const char *Data() const { return dataBuffer.data(); }

  std::size_t Length() const { return HeaderLength + bodyLength; }

  const char *Body() const { return dataBuffer.data() + HeaderLength; }

  const int BodyLength() { return bodyLength; }

  static bool DecodeHeader(const char * buffer, int * bodyLength, MsgType * frameType) {
    if (static_cast<unsigned char>(buffer[0]) == 0 &&
        static_cast<unsigned char>(buffer[1]) == 0xFF &&
        static_cast<unsigned char>(buffer[2]) == 0 &&
        static_cast<unsigned char>(buffer[3]) == 0) {
      *bodyLength =
          static_cast<int>(static_cast<unsigned char>(buffer[4]) << 24 |
                           static_cast<unsigned char>(buffer[5]) << 16 |
                           static_cast<unsigned char>(buffer[6]) << 8 |
                           static_cast<unsigned char>(buffer[7]));
      *frameType = MsgType(
          static_cast<int>(static_cast<unsigned char>(buffer[8]) << 24 |
                           static_cast<unsigned char>(buffer[9]) << 16 |
                           static_cast<unsigned char>(buffer[10]) << 8 |
                           static_cast<unsigned char>(buffer[11])));
      // ignore timestamp
      if (*bodyLength > MaxBodyLength) {
        *bodyLength = 0;
        return false;
      }
      return true;
    } else {
      return false;
    }
  }

private:
  std::vector<char> dataBuffer;
  std::size_t bodyLength;
};

#endif

