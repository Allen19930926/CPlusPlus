// Copyright 2020 Horizon Robotics.

#pragma once

#include <iostream>
#include <memory>

#include "common/define.h"
#include "protocol/camera.pb.h"
#include "protocol/can.pb.h"
#include "protocol/frame.v2.pb.h"
#include "protocol/ihbc_output.pb.h"
#include "protocol/image.pb.h"
#include "protocol/lane.pb.h"
#include "protocol/parsing.pb.h"
#include "protocol/lane.v2.pb.h"
#include "protocol/line.v2.pb.h"
#include "protocol/object.pb.h"
#include "protocol/obstacle.pb.h"
#include "protocol/obstacle_raw.pb.h"
#include "protocol/perception_base.pb.h"
#include "protocol/runtime.pb.h"
#include "protocol/runtime_env.pb.h"
#include "protocol/scan_point.pb.h"
#include "protocol/skeleton.pb.h"
#include "protocol/odometry.pb.h"
#include "protocol/box_3D_raw.pb.h"
#include "protocol/online_calib_msg.pb.h"
#include "protocol/isp_param.pb.h"

void ParseImageMsg(std::shared_ptr<google::protobuf::Message> message,
                   const matrix_sample::BlockDese &block);
void ParseCameraMsg(std::shared_ptr<google::protobuf::Message> message,
                    const matrix_sample::BlockDese &block);
void ParseIHBCMsg(std::shared_ptr<google::protobuf::Message> message,
                  const matrix_sample::BlockDese &block);
void ParseLaneV2Msg(std::shared_ptr<google::protobuf::Message> message,
                    const matrix_sample::BlockDese &block);
void ParseLineV2Msg(std::shared_ptr<google::protobuf::Message> message,
                    const matrix_sample::BlockDese &block);
void ParseObstacleMsg(std::shared_ptr<google::protobuf::Message> message,
                      const matrix_sample::BlockDese &block);
void ParseObstacleRawMsg(std::shared_ptr<google::protobuf::Message> message,
                         const matrix_sample::BlockDese &block);
void ParseCanMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block);
void ParsePedResultMsg(std::shared_ptr<google::protobuf::Message> message,
                    const matrix_sample::BlockDese &block);
void ParseROIMsg(std::shared_ptr<google::protobuf::Message> message,
                   const matrix_sample::BlockDese &block);
void ParseRuntimeMsg(std::shared_ptr<google::protobuf::Message> message,
                       const matrix_sample::BlockDese &block);
void ParseRuntimeEnvMsg(std::shared_ptr<google::protobuf::Message> message,
                          const matrix_sample::BlockDese &block);
void ParseScanPointsMsg(std::shared_ptr<google::protobuf::Message> message,
                          const matrix_sample::BlockDese &block);
void ParseSkeletonRawsMsg(std::shared_ptr<google::protobuf::Message> message,
                       const matrix_sample::BlockDese &block);
void ParseTrafficlightMsg(std::shared_ptr<google::protobuf::Message> message,
                       const matrix_sample::BlockDese &block);
void ParseOdometryMsg(std::shared_ptr<google::protobuf::Message> message,
                       const matrix_sample::BlockDese &block);
void ParseVehicleresultMsg(std::shared_ptr<google::protobuf::Message> message,
                       const matrix_sample::BlockDese &block);
void ParseBoxes3drawMsg(std::shared_ptr<google::protobuf::Message> message,
                       const matrix_sample::BlockDese &block);
void ParseLaneresultMsg(std::shared_ptr<google::protobuf::Message> message,
                       const matrix_sample::BlockDese &block);
void ParseLaneparsingMsg(std::shared_ptr<google::protobuf::Message> message,
                       const matrix_sample::BlockDese &block);
void ParseParsingMsg(std::shared_ptr<google::protobuf::Message> message,
                       const matrix_sample::BlockDese &block);
void ParseOnlinecalibMsg(std::shared_ptr<google::protobuf::Message> message,
                       const matrix_sample::BlockDese &block);
void ParseIspparamMsg(std::shared_ptr<google::protobuf::Message> message,
                       const matrix_sample::BlockDese &block);
void ParseObjectMsg(std::shared_ptr<google::protobuf::Message> message,
                       const matrix_sample::BlockDese &block);