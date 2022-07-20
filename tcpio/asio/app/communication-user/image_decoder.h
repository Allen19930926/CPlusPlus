// Copyright 2020 Horizon Robotics.

#pragma once
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "common/util.h"
#include "common/define.h"
#include "opencv2/opencv.hpp"
#include "protocol/image.pb.h"

#ifdef H265_DECODER_ENABLE
#include "common/ffmpeg_decode.h"
#endif

namespace matrix_sample {

class ImageDecoder {
 public:
  void DecodeImages(const ImageProto::Image &meta, const uint8_t *data,
                    uint64_t size, int64_t gen_ts);

 private:
  void WriteData(const std::string &filename, const uint8_t *data,
                 uint64_t size);

  std::string MakeImageDir(const std::string &upperDir);

 private:
#ifdef H265_DECODER_ENABLE
  matrix_sample::DecoderH265 av_decoder_;
#endif
};

}  // namespace matrix_sample
