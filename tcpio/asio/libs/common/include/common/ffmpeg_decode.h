// Copyright 2020 Horizon Robotics.
#ifndef SRC_COMMON_FFMPEG_DECODE_H_
#define SRC_COMMON_FFMPEG_DECODE_H_

#include <opencv2/opencv.hpp>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

namespace matrix_sample {
class DecoderH265 {
 public:
  DecoderH265();
  ~DecoderH265();
  bool Init();
  bool Destroy();
  cv::Mat Decode(uint8_t *data, uint32_t data_size);
 private:
  const AVCodec *codec;
  AVCodecContext *codec_context;
  AVFrame *avframe_yuv;
};
#endif  // SRC_COMMON_FFMPEG_DECODE_H_
}  // namespace matrix_sample
