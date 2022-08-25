// Copyright 2021 Horizon Robotics

#include "common/ffmpeg_decode.h"

#include <iostream>
#include <string>

namespace matrix_sample {

DecoderH265::DecoderH265() { Init(); }
DecoderH265::~DecoderH265() { Destroy(); }

bool DecoderH265::Init() {
  avcodec_register_all();
  codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
  codec_context = avcodec_alloc_context3(codec);
  if (avcodec_open2(codec_context, codec, NULL) < 0) {
    std::cout << "Could not open codec" << std::endl;
    return false;
  }
  avframe_yuv = av_frame_alloc();
  if (!avframe_yuv) {
    std::cout << "Could not allocate video frame" << std::endl;
    return false;
  }
  return true;
}
bool DecoderH265::Destroy() {
  avcodec_close(codec_context);
  av_free(codec_context);
  av_frame_free(&avframe_yuv);
  return true;
}

cv::Mat DecoderH265::Decode(uint8_t* data, uint32_t data_size) {
  AVPacket avpkt_h265;
  av_init_packet(&avpkt_h265);
  avpkt_h265.data = data;
  avpkt_h265.size = data_size;
  if (avcodec_send_packet(codec_context, &avpkt_h265)) {
    std::cout << "avcodec_send_packet fail" << std::endl;
    return cv::Mat();
  }
  if (avcodec_receive_frame(codec_context, avframe_yuv)) {
    std::cout << "avcodec_receive_frame fail" << std::endl;
    return cv::Mat();
  }

  int h = avframe_yuv->height;
  int w = avframe_yuv->width;
  uint8_t* yuv_buf = new uint8_t[h * w * 3];

  cv::Mat mat_rgb;
  mat_rgb.create(avframe_yuv->height, avframe_yuv->width, CV_8UC3);

  for (int i = 0; i < h; ++i) {
    memcpy(yuv_buf + w * i, avframe_yuv->data[0] + avframe_yuv->linesize[0] * i,
           w);
  }

  for (int i = 0; i < h / 2; ++i) {
    memcpy(yuv_buf + w * h + w / 2 * i,
           avframe_yuv->data[1] + avframe_yuv->linesize[1] * i, w / 2);
  }

  for (int i = 0; i < h / 2; ++i) {
    memcpy(yuv_buf + w * h + w / 2 * (h / 2) + w / 2 * i,
           avframe_yuv->data[2] + avframe_yuv->linesize[2] * i, w / 2);
  }

  cv::Mat mat_yuv(h + h / 2, w, CV_8UC1, yuv_buf);

  cv::cvtColor(mat_yuv, mat_rgb, cv::COLOR_YUV420p2RGB);

  delete yuv_buf;

  return mat_rgb;
}
}  // namespace matrix_sample
