// Copyright 2020 Horizon Robotics.
#include "util.h"  // NOLINT

#include <chrono>
#include <fstream>
#include <iostream>

#ifndef _WIN32
#include <arpa/inet.h>
#include <net/if.h>
#include <pthread.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#else
#include <direct.h>
#include <winsock2.h>
#endif

namespace matrix_sample {
namespace util {

int RLEDecompress(const uint8_t *input_data, int size, uint8_t *output_data,
                  int &buf_len) {
  const uint8_t *ptr = input_data;

  if (ptr[0] != 0xFF || ptr[1] != 'R' || ptr[2] != 'L' || ptr[3] != 'E') {
    std::cout << "RLE header mismatch !!!" << std::endl;
    return -1;
  }

  // Skip the magic number first
  ptr += 4;
  // Then the height and width
  uint16_t height = (*ptr++ << 8) & 0xFF00;
  height |= static_cast<uint16_t>(*ptr++);
  uint16_t width = (*ptr++ << 8) & 0xFF00;
  width |= static_cast<uint16_t>(*ptr++);

  if (height <= 0 || width <= 0) {
    return -1;
  }

  if (buf_len < height * width) {
    return -2;
  }

  buf_len = height * width;

  // Then the data
  int rle_cnt = (size - 8) / 3;
  uint8_t *p_im_data = output_data;
  for (int i = 0; i < rle_cnt; ++i) {
    uint8_t label = *ptr++;
    uint16_t cnt = *reinterpret_cast<const uint16_t *>(ptr);
    ptr += 2;

    for (int j = 0; j < cnt; ++j) {
      *p_im_data++ = label;
    }
  }

  return 0;
}

std::string GetIPFromZMQEndpoint(const std::string endpoint) {
  auto start_index = endpoint.find_last_of('/');
  auto end_index = endpoint.rfind(':');
  if (start_index == std::string::npos || end_index == std::string::npos ||
      start_index + 1 >= end_index) {
    return std::string();
  } else {
    start_index += 1;
    return endpoint.substr(start_index, end_index - start_index);
  }
}

int64_t GetTimeStamp() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

bool IsJpegStream(uint8_t *data, int len) {
  if (data[0] == 0xFF && data[1] == 0xD8 && data[2] == 0xFF &&
      data[3] == 0xE0 && data[len - 2] == 0xFF && data[len - 1] == 0xD9) {
    return true;
  } else {
    return false;
  }
}

std::string GetCurrentDir() {
#ifdef _WIN32
  std::string path = _getcwd(NULL, 0);
#else
  std::string path = getcwd(NULL, 0);
#endif

  std::string::size_type pos(0);
  while ((pos = path.find("\\", pos)) != std::string::npos) {
    path.replace(pos, 1, "/");
  }
  return path;
}

int GenerateFile(const std::string &file_name, const std::string &content) {
  if (file_name.empty()) {
    return 0;
  }

  std::ofstream ofs;

  ofs.open(file_name, std::ios::out);

  if (!ofs.is_open()) {
    return 0;
  }

  ofs << content;

  return content.size();
}

}  // namespace util
}  // namespace matrix_sample
