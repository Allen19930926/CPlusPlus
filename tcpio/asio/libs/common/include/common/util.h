// Copyright 2020 Horizon Robotics.
#ifndef SRC_COMMON_UTIL_H_
#define SRC_COMMON_UTIL_H_

#include <cstdint>
#include <string>

namespace matrix_sample {
namespace util {

/**
 * @brief RLE decompress
 * @details this function do RLE decompress for the input data
 * @param input_data RLE compressed data
 * @param size length of input compressed data
 * @param output_data pointer to output data
 * @param buf_len length of decompressed data
 * @return success or not
 *  @retval 0 success
 *  @retval < 0 failed
 */
int RLEDecompress(const uint8_t *input_data, int size, uint8_t *output_data,
                  int &buf_len);

/**
 * @brief Get IP address from zmq endpoint
 * @param zmq endpoint
 * @return IP address string.
 */
std::string GetIPFromZMQEndpoint(const std::string endpoint);

/**
 * @brief Generate a timestamp tag
 * @return timestamp in millsecond.
 */
int64_t GetTimeStamp();

/**
 * @brief discriminate a data buffer is jpeg stream or not.
 * @param data, point to the data buffer
 * @param len, length of the data
 * @return result of the discrimination
 *   @retval true, data is jpeg stream, otherwise
 *   @retval false.
 */
bool IsJpegStream(uint8_t *data, int len);

/**
 * @brief Generate current dir str
 * @return current dir.
 */
std::string GetCurrentDir();

/**
 * @brief Generate file
 * @param file_name, generate file name
 * @param content, generate file content
 * @return content length.
 */
int GenerateFile(const std::string &file_name, const std::string &content);

}  // namespace util
}  // namespace matrix_sample

#endif  // SRC_COMMON_UTIL_H_
