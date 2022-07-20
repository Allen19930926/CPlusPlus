// Copyright 2021 Horizon Robotics

#include "image_decoder.h"

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include <fstream>

#include "opencv2/imgproc.hpp"

namespace matrix_sample {

void ImageDecoder::DecodeImages(const ImageProto::Image &meta,
                                const uint8_t *data, uint64_t size,
                                int64_t gen_ts) {
  if (!data || size == 0) {
    return;
  }

  auto format = meta.format();
  auto comp = meta.compression_format();
  auto width = meta.width();
  auto height = meta.height();

  std::cout << "format: " << format << ", "
            << "compression format: " << comp << ", "
            << "width: " << width << ", "
            << "height: " << height << std::endl;

  std::string filename = MakeImageDir(matrix_sample::util::GetCurrentDir()) +
                         std::string("image_") + std::to_string(gen_ts) +
                         ".jpeg";

  if (format == ImageProto::BITSTREAM) {
#ifdef H265_DECODER_ENABLE
    cv::Mat nv12 = av_decoder_.Decode(const_cast<uint8_t *>(data), size);
    if (!nv12.data) {
      return;
    }
    // TODO( ): need to check the format of AVFrame
    cv::imwrite(filename, nv12);
#endif
  } else {
    bool is_jpeg = matrix_sample::util::IsJpegStream(
        const_cast<uint8_t *>(data), static_cast<int>(size));

    if (is_jpeg || format == ImageProto::JPEG) {
      WriteData(filename, data, size);
    } else if (format == ImageProto::NV12) {
      cv::Mat dst;
      cv::Mat src(height * 3 / 2, width, CV_8UC1, const_cast<uint8_t *>(data));
      cv::cvtColor(src, dst, cv::COLOR_YUV2BGR_NV12);
      cv::imwrite(filename, dst);
    } else if (format == ImageProto::YV12) {
      cv::Mat dst;
      cv::Mat src(height * 3 / 2, width, CV_8UC1, const_cast<uint8_t *>(data));
      cv::cvtColor(src, dst, cv::COLOR_YUV2RGB_YV12);
      cv::imwrite(filename, dst);
    }
  }
}

void ImageDecoder::WriteData(const std::string &filename, const uint8_t *data,
                             uint64_t size) {
  std::ofstream ofs(filename, std::ios_base::binary);
  ofs.write(reinterpret_cast<const char *>(data), size);
  ofs.close();
}

std::string ImageDecoder::MakeImageDir(const std::string &upperDir) {
  std::string saveDir = upperDir + "/Images";

#ifdef _WIN32
  _mkdir(saveDir.c_str());
#else
  mkdir(saveDir.c_str(), 0777);
#endif

  return saveDir + "/";
}

}  // namespace matrix_sample
