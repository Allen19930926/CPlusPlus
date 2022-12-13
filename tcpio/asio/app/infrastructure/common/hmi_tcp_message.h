/*
 * @Descripttion: 
 * @version: 
 * @Author: congsir
 * @Date: 2022-12-01 15:11:10
 * @LastEditors: 
 * @LastEditTime: 2022-12-02 09:41:00
 */
#ifndef HMI_TCP_MESSAGE_H
#define HMI_TCP_MESSAGE_H

#include <cstddef>
#include <cstring>
#include <string>
class HmiTcpMessage {
public:
  static constexpr std::size_t HeaderLength = 8;
  static constexpr std::size_t MaxBodyLength = 4000;

  HmiTcpMessage() : bodyLength(0) {}

  HmiTcpMessage(const std::string& data) {
    bodyLength = data.length();
    dataBuffer[0] = 0;
    dataBuffer[1] = 0xFF;
    dataBuffer[2] = 0;
    dataBuffer[3] = 0;
    dataBuffer[4] = static_cast<unsigned char>(bodyLength >> 24);
    dataBuffer[5] = static_cast<unsigned char>(bodyLength >> 16);
    dataBuffer[6] = static_cast<unsigned char>(bodyLength >> 8);
    dataBuffer[7] = static_cast<unsigned char>(bodyLength);
    memset(dataBuffer + HeaderLength, 0, MaxBodyLength);
    memcpy(dataBuffer + HeaderLength, data.c_str(), data.length() + 1);
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
      if (bodyLength > MaxBodyLength) {
        bodyLength = 0;
        return false;
      }
      return true;
    } 
    else 
    {
      return false;
    }
  }

private:
  char dataBuffer[HeaderLength + MaxBodyLength];
  std::size_t bodyLength;
};

#endif

