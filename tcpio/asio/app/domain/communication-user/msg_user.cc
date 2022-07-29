// Copyright 2020 Horizon Robotics.

#include "msg_user.h"

#include "image_decoder.h"
#include "msg_parser.h"

namespace matrix_sample {

void MsgUser::RecvMsg(MsgFrame *frame) {
  std::unique_lock<std::mutex> lck(wait_mutex_);
  msg_queue_.push(*frame);
  wait_cond_.notify_one();
}

void MsgUser::HandleMsg() {
  if (start_thread_) {
    std::cout << "Sub Thread is running ..." << std::endl;
    return;
  }

  std::cout << "Sub thread running ->" << std::endl;
  start_thread_ = true;
  sub_thread_ = std::thread([this]() {
    while (start_thread_) {
      ProcessMsg();
    }
  });
}

void MsgUser::HandleTopic(const Blocks &blocks, const std::string topic,
                          Handler handler) {
  for (auto &bl : blocks) {
    auto msg = factory_.MakeProtobufMsg(topic);
    if (msg) {
        handler(msg, bl);
    }
  }
}

void MsgUser::ProcessMsg() {
  std::unique_lock<std::mutex> lck(wait_mutex_);
  wait_cond_.wait(lck, [this]() { return !msg_queue_.empty(); });

  if (msg_queue_.empty()) {
    return;
  }
  auto msg = msg_queue_.front();
  msg_queue_.pop();
  // DLOG(INFO) << "process message";

  for (auto &it : msg) {
    if (it.topic == "image#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseImageMsg);
    } else if (it.topic == "ihbc#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseIHBCMsg);
    } else if (it.topic == "traffic_light#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseTrafficlightMsg);
    } else if (it.topic == "vehicle_result#0") {
      HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
                  ParseVehicleresultMsg);
    } else if (it.topic == "boxes_3d_raw#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseBoxes3drawMsg);
    } else if (it.topic == "camera_default#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseCameraMsg);
    } else if (it.topic == "camera_runtime#0") {
    } else if (it.topic == "can#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseCanMsg);
    } else if (it.topic == "contour_endpts#0") {
    } else if (it.topic == "crop_roi#0") {
    } else if (it.topic == "isp_param#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseIspparamMsg);
    } else if (it.topic == "lane_parsing#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseLaneparsingMsg);
    } else if (it.topic == "lane_result#0") {
      HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
                  ParseLaneresultMsg);
    } else if (it.topic == "laneline_v2#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseLineV2Msg);
    } else if (it.topic == "multi_cam_state#0") {
    } else if (it.topic == "obstacle#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseObstacleMsg);
    } else if (it.topic == "obstacles_raw#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseObstacleRawMsg);
    } else if (it.topic == "odometry#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseOdometryMsg);
    } else if (it.topic == "online_calib#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseOnlinecalibMsg);
    } else if (it.topic == "parsing#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseParsingMsg);
    } else if (it.topic == "ped_result#0") {
      HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
                  ParsePedResultMsg);
    } else if (it.topic == "resize_roi#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseROIMsg);
    } else if (it.topic == "runtime#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseRuntimeMsg);
    } else if (it.topic == "runtime_env#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseRuntimeEnvMsg);
    } else if (it.topic == "scan_points#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseScanPointsMsg);
    } else if (it.topic == "skeleton_raw#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseSkeletonRawsMsg);
    } else if (it.topic == "object#0") {
      // HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
      //             ParseObjectMsg);
    } else {
    }
  }
  for (auto &it : msg) {
    if (it.topic == "laneline_v2#0") {
      HandleTopic(it.block, (it.topic).substr(0, (it.topic).rfind('#')),
                  ParseLineV2Msg);
    }
  }
}
}  // namespace matrix_sample
