// Copyright 2020 Horizon Robotics.

#include "msg_parser.h"

#include <string>

#include "can_decoder.h"
#include "image_decoder.h"
#include "event_queue.h"
#include "event_msg.h"
#include "hb_data.h"
#include <map>
#include <glog/logging.h>

std::map<int, float> lifetime;

static const std::string ImageFormat(::google::protobuf::int32 fmt) {
  switch (fmt) {
    case ImageProto::GRAY:
      return "GRAY";
    case ImageProto::YV12:
      return "YV12";
    case ImageProto::JPEG:
      return "JPEG";
    case ImageProto::PNG:
      return "PNG";
    case ImageProto::CR12:
      return "CR12";
    case ImageProto::BAD:
      return "BAD";
    case ImageProto::NV12:
      return "NV12";
    case ImageProto::NV21:
      return "NV21";
    case ImageProto::TIMEOUT:
      return "TIMEOUT";
    case ImageProto::BYPASS_ONLY:
      return "BYPASS_ONLY";
    case ImageProto::YUYV:
      return "YUYV";
    case ImageProto::UYVY:
      return "UYVY";
    case ImageProto::RAW12_PLANAR:
      return "RAW12_PLANAR";
    case ImageProto::RAW12_PLANAR_JPEG:
      return "RAW12_PLANAR_JPEG";
    case ImageProto::RAW12_PLANAR_JPEG_LOSSLESS:
      return "RAW12_PLANAR_JPEG_LOSSLESS";
    case ImageProto::RAW12:
      return "RAW12";
    case ImageProto::RAW12_JPEG:
      return "RAW12_JPEG";
    case ImageProto::RAW12_JPEG_LOSSLESS:
      return "RAW12_JPEG_LOSSLESS";
    case ImageProto::RAW16:
      return "RAW16";
    case ImageProto::RAW16_JPEG:
      return "RAW16_JPEG";
    case ImageProto::RAW16_JPEG_LOSSLESS:
      return "RAW16_JPEG_LOSSLESS";
    case ImageProto::RAW20:
      return "RAW20";
    case ImageProto::RAW20_JPEG:
      return "RAW20_JPEG";
    case ImageProto::RAW20_JPEG_LOSSLESS:
      return "RAW20_JPEG_LOSSLESS";
    case ImageProto::RAW20_PLANAR:
      return "RAW20_PLANAR";
    case ImageProto::RAW20_PLANAR_JPEG:
      return "RAW20_PLANAR_JPEG";
    case ImageProto::RAW20_PLANAR_JPEG_LOSSLESS:
      return "RAW20_PLANAR_JPEG_LOSSLESS";
    case ImageProto::RAW16_WITH_FRONT:
      return "RAW16_WITH_FRONT";
    case ImageProto::RAW16_WITH_REAR:
      return "RAW16_WITH_REAR";
    case ImageProto::RAW16_WITH_FRONT_REAR:
      return "RAW16_WITH_FRONT_REAR";
    case ImageProto::RAW16_PLANAR_JPEG_LOSSLESS:
      return "RAW16_PLANAR_JPEG_LOSSLESS";
    case ImageProto::RAW14_PLANAR_JPEG_LOSSLESS:
      return "RAW14_PLANAR_JPEG_LOSSLESS";
    case ImageProto::BITSTREAM:
      return "BITSTREAM";
    default:
      return "Unknown";
  }
}


static void ParseObstacles(const ObstacleProto::Obstacles *obstacles, gohigh::Obstacles * const result)
{

  // auto out = gohigh::Obstacles();
  memset((void*)result, 0, sizeof(result));

  LOG(INFO) << "execute parse J3 data";

  result->cipv_id = obstacles->cipv_id();
  result->mcp_id = obstacles->mcp_id();
  result->obstacle_num = obstacles->obstacle().size();
  auto * obs = &result->obstacles[0];
  for (uint32_t i = 0; i < result->obstacle_num; i++)
  {
    auto & obstacle = obstacles->obstacle(i);
    obs = &result->obstacles[i];
    obs->id = obstacle.id();
    obs->timestamp = obstacles->header().time_stamp();
    obs->type = obstacle.type();
    obs->conf = obstacle.conf();
    obs->life_time = obstacle.life_time();
    obs->serial_number = obstacle.serial_number();
    obs->select_level = obstacle.select_level();
    obs->world_info.yaw = obstacle.world_info().yaw();
    obs->world_info.vel.vx = obstacle.world_info().vel().vx();
    obs->world_info.vel.vy = obstacle.world_info().vel().vy();
    obs->world_info.width = obstacle.world_info().width();
    obs->world_info.height = obstacle.world_info().height();
    obs->world_info.position.x = obstacle.world_info().position().x();
    obs->world_info.position.y = obstacle.world_info().position().y();
    obs->world_info.ttc = obstacle.world_info().ttc();
    obs->world_info.curr_lane = obstacle.world_info().curr_lane();
    obs->world_info.ettc = obstacle.world_info().ettc();
    obs->world_info.acc = obstacle.world_info().acc();
    obs->world_info.motion_state = obstacle.world_info().motion_state();
    obs->world_info.vel_abs_world.vx = obstacle.world_info().vel_abs_world().vx();
    obs->world_info.vel_abs_world.vy = obstacle.world_info().vel_abs_world().vy();
    obs->world_info.acc_abs_world.ax = obstacle.world_info().acc_abs_world().ax();
    obs->world_info.acc_abs_world.ay = obstacle.world_info().acc_abs_world().ay();
    obs->world_info.motion_category = obstacle.world_info().motion_category();
    obs->world_info.position_type = obstacle.world_info().position_type();
    obs->world_info.yaw_rate = obstacle.world_info().yaw_rate();
    obs->world_info.sigma_yaw = obstacle.world_info().sigma_yaw();
    for (uint32_t j = 0; j < obstacle.world_info().sigma_vel_size(); j++)
    {
      obs->world_info.sigma_vel[j] = obstacle.world_info().sigma_vel(j);
    }
    
    obs->world_info.sigma_width = obstacle.world_info().sigma_width();
    obs->world_info.sigma_height = obstacle.world_info().sigma_height();
    for (uint32_t j = 0; j < obstacle.world_info().sigma_position_size(); j++)
    {
      obs->world_info.sigma_position[j] = obstacle.world_info().sigma_position(j);
    }
    obs->world_info.sigma_length = obstacle.world_info().sigma_length();
    obs->world_info.conf_yaw = obstacle.world_info().conf_yaw();
    obs->world_info.cipv = obstacle.world_info().cipv();
    obs->world_info.measurement_status = obstacle.world_info().measurement_status();
    obs->world_info.mid_angle = obstacle.world_info().mid_angle();
    obs->world_info.obj_corner_point.objCornerPoint_x = obstacle.world_info().obj_corner_point().objcornerpoint_x();
    obs->world_info.obj_corner_point.objCornerPoint_y = obstacle.world_info().obj_corner_point().objcornerpoint_y();
    obs->world_info.obj_corner_point.objDistInLane = obstacle.world_info().obj_corner_point().objdistinlane();
    obs->world_info.obj_corner_point.objCutInFlag = obstacle.world_info().obj_corner_point().objcutinflag();
    obs->world_info.obj_corner_point.objCutInLane = obstacle.world_info().obj_corner_point().objcutinlane();
    obs->world_info.obj_corner_point.ll_type = obstacle.world_info().obj_corner_point().ll_type();
    obs->world_info.obj_corner_point.rr_type = obstacle.world_info().obj_corner_point().rr_type();
    obs->world_info.obj_corner_point.distance_to_ll = obstacle.world_info().obj_corner_point().distance_to_ll();
    obs->world_info.obj_corner_point.distance_to_rr = obstacle.world_info().obj_corner_point().distance_to_rr();
    obs->world_info.acc_ref.ax = obstacle.world_info().acc_ref().ax();
    obs->world_info.acc_ref.ay = obstacle.world_info().acc_ref().ay();


    LOG(INFO) << " parse j3 vehicle.De_Timestamp_u32          = raw:" << obstacle.life_time() << " 32:" << obs->timestamp
    << "\n\t parse j3 vehicle.De_ID_u8                  = " << obs->id
    << "\n\t parse j3 vehicle.De_dx_f32                 = " << obs->world_info.position.x
    << "\n\t parse j3 vehicle.De_dy_f32                 = " << obs->world_info.position.y
    << "\n\t parse j3 vehicle.De_vx_f32                 = " << obs->world_info.vel_abs_world.vx
    << "\n\t parse j3 vehicle.De_vy_f32                 = " << obs->world_info.vel_abs_world.vx;
  }
}

void ParseImageMsg(std::shared_ptr<google::protobuf::Message> message,
                   const matrix_sample::BlockDese &block) {
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));

  ImageProto::Image *image = static_cast<ImageProto::Image *>(message.get());

  std::cout << "Image -> {"
            << "format: " << ImageFormat(image->format()) << ", "
            << "width: " << image->width() << ", "
            << "height: " << image->height() << ", "
            << "channel: " << image->channel() << "}" << std::endl;
  std::cout << std::endl;

  // parse image

  // matrix_sample::ImageDecoder image_decoder;
  // image_decoder.DecodeImages(*image, block.data_->data(), block.data_size_,
  //                            block.gen_ts_);
}

void ParseCameraMsg(std::shared_ptr<google::protobuf::Message> message,
                    const matrix_sample::BlockDese &block) {
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));

  CameraProto::CameraParam *camera =
      static_cast<CameraProto::CameraParam *>(message.get());

  std::cout << "Camera -> {"
            << "camera_id: " << camera->cam_id() << ", "
            << "frame_id: " << camera->frame_id() << ", "
            << "camera_x: " << camera->camera_x() << ", "
            << "camera_y: " << camera->camera_y() << ", "
            << "camera_z: " << camera->camera_z() << ", "
            << "pitch: " << camera->pitch() << ", "
            << "yaw: " << camera->yaw() << ", "
            << "roll: " << camera->roll() << "}" << std::endl;
  std::cout << std::endl;
}

void ParseIHBCMsg(std::shared_ptr<google::protobuf::Message> message,
                  const matrix_sample::BlockDese &block) {
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));

  IHBCResultProto::IHBCResultProto *ihbc =
      static_cast<IHBCResultProto::IHBCResultProto *>(message.get());

  std::cout << "IHBC -> {"
            << "cal_lux_up: " << ihbc->cal_lux_up() << ", "
            << "light_on: " << ihbc->light_on() << "}" << std::endl;
  std::cout << std::endl;
}

void ParseLaneV2Msg(std::shared_ptr<google::protobuf::Message> message,
                    const matrix_sample::BlockDese &block) {
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));

  lane_v2::Lane *lane = static_cast<lane_v2::Lane *>(message.get());

  std::cout << "Lane -> {"
            << "id: " << lane->id() << ", "
            << "str_id: " << lane->str_id() << ", " << std::endl;

  std::cout << "l_laneline_ids: [";
  for (auto &ll : lane->l_laneline_ids()) {
    std::cout << ll << ", ";
  }
  std::cout << "], " << std::endl;

  std::cout << "r_laneline_ids: [";
  for (auto &rl : lane->r_laneline_ids()) {
    std::cout << rl << ", ";
  }
  std::cout << "], " << std::endl;

  std::cout << "driveline_id: " << lane->driveline_id() << ", "
            << "direction: " << lane->direction() << ", "
            << "transition: " << lane->transition() << ", "
            << "lane_length: " << lane->lane_length() << ", "
            << "type: " << lane->type() << ", " << std::endl;

  // LaneAttr
  std::cout << "attrs: [";
  for (auto &la : lane->attrs()) {
    std::cout << "{curvature: " << la.curvature() << ", "
              << "slope: " << la.slope() << ", "
              << "banking: " << la.banking() << ", "
              << "headingAngle: " << la.headingangle() << ", "
              << "offset: " << la.offset() << ", "
              << "width: " << la.width() << "}, ";
  }
  std::cout << "], " << std::endl;

  // Link Objs
  std::cout << "Link Objs: [";
  for (auto &o : lane->objs()) {
    std::cout << "{id: " << o.id() << ", "
              << "offset: " << o.offset() << ", "
              << "end_offset: " << o.end_offset() << "}, ";
  }
  std::cout << "], " << std::endl;

  // Link obstacles
  std::cout << "Link obstacles: [";
  for (auto &ob : lane->obstacles()) {
    std::cout << "{id: " << ob.id() << ", "
              << "offset: " << ob.offset() << ", "
              << "end_offset: " << ob.end_offset() << "}, ";
  }
  std::cout << "], " << std::endl;

  // pred_ids
  std::cout << "pred_ids: [";
  for (auto &pi : lane->pred_ids()) {
    std::cout << pi << ", ";
  }
  std::cout << "], " << std::endl;

  // succ_ids
  std::cout << "succ_ids: [";
  for (auto &si : lane->succ_ids()) {
    std::cout << si << ", ";
  }
  std::cout << "], " << std::endl;

  // left_ids
  std::cout << "left_ids: [";
  for (auto &li : lane->left_ids()) {
    std::cout << li << ", ";
  }
  std::cout << "], " << std::endl;

  // right_ids
  std::cout << "right_ids: [";
  for (auto &ri : lane->right_ids()) {
    std::cout << ri << ", ";
  }
  std::cout << "], " << std::endl;

  // restrictions
  std::cout << "restrictions: [" << std::endl;
  for (auto &res : lane->restrictions()) {
    // vehicle_types
    std::cout << "{vehicle_types: [";
    for (auto &vt : res.vehicle_types()) {
      std::cout << "{type: " << vt.type() << ", "
                << "conf: " << vt.conf() << "}, ";
    }
    std::cout << "], " << std::endl;

    // speed_limits
    std::cout << "speed_limits: [";
    for (auto &sl : res.speed_limits()) {
      std::cout << "{limit_type: " << sl.limit_type() << ", "
                << "speed_value: " << sl.speed_value() << ", "
                << "source: " << sl.source() << ", "
                << "offset: " << sl.offset() << ", "
                << "end_offset: " << sl.end_offset() << "}, ";
    }
    std::cout << "], " << std::endl;

    // time_limits
    std::cout << "time_limits: [";
    for (auto &tl : res.time_limits()) {
      std::cout << "{time_begin: " << tl.time_begin() << ", "
                << "time_end: " << tl.time_end() << "}, ";
    }
    std::cout << "], " << std::endl;

    // lanemarking_types
    std::cout << "lanemarking_types: [";
    for (auto &lt : res.lanemarking_types()) {
      std::cout << lt << ", ";
    }
    std::cout << "], " << std::endl;

    // trafficsign_types
    std::cout << "trafficsign_types: [";
    for (auto &tt : res.trafficsign_types()) {
      std::cout << tt << ", ";
    }
    std::cout << "], " << std::endl;

    std::cout << "weight_limit: " << res.weight_limit() << ", "
              << "height_limit: " << res.height_limit() << "}, " << std::endl;
  }
  std::cout << "], " << std::endl;

  // ** bounding_polygon beginning **
  auto &bp = lane->bounding_polygon();
  std::cout << "bounding_polygon: {" << std::endl;
  // points
  std::cout << "points: [";
  for (auto &p : bp.points()) {
    std::cout << "{x: " << p.x() << ", "
              << "y: " << p.y() << ", "
              << "z: " << p.z() << ", " << std::endl;
    std::cout << "cov: [";
    for (auto &cov : p.cov()) {
      std::cout << cov << ", ";
    }
    std::cout << "], }, " << std::endl;
  }
  std::cout << "], " << std::endl;

  // normal
  std::cout << "normal: {"
            << "x: " << bp.normal().x() << ", "
            << "y: " << bp.normal().y() << ", "
            << "z: " << bp.normal().z() << ", " << std::endl;
  std::cout << "cov: [";
  for (auto &cov : bp.normal().cov()) {
    std::cout << cov << ", ";
  }
  std::cout << "], }, " << std::endl;

  // orientation
  std::cout << "orientation: {"
            << "x: " << bp.orientation().x() << ", "
            << "y: " << bp.orientation().y() << ", "
            << "z: " << bp.orientation().z() << ", " << std::endl;
  std::cout << "cov: [";
  for (auto &cov : bp.orientation().cov()) {
    std::cout << cov << ", ";
  }
  std::cout << "], }, " << std::endl;

  // edgeline_width
  std::cout << "edgeline_width: " << bp.edgeline_width() << ", ";
  std::cout << "}" << std::endl;

  // ** bounding_polygon ending **
  std::cout << "}" << std::endl;

  std::cout << std::endl;
}

void ParseLineV2Msg(std::shared_ptr<google::protobuf::Message> message,
                    const matrix_sample::BlockDese &block) {
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));

  line_v2::Lines *lines = static_cast<line_v2::Lines *>(message.get());
  if (lines->lines_size() == 0)
    return;

  // std::cout << "LineV2 -> {";
  // std::cout << "Line: [";
  // for (auto &l : lines->lines()) {
  //   std::cout << "{id: " << l.id() << ", "
  //             << "str_id: " << l.str_id() << ", "
  //             << "type: " << l.type() << ", "
  //             << "source: " << l.source() << ", "
  //             << "position: " << l.position() << ", "
  //             << "}, " << std::endl;
  // }
  // std::cout << "], " << std::endl;
  // std::cout << "}" << std::endl;
  // std::cout << std::endl;
    auto print = [](const gohigh::Line * const line) {
      LOG(INFO) << "Line [ id: " << line->id
      << "\n\t valid: " << line->valid
      << "\n\t life_time: " << line->life_time
      << "\n\t type: " << line->type
      << "\n\t conf: " << line->conf
      << "\n\t width: " << line->width 
      << "\n\t start_pt: [" << line->start_pt.x << " " << line->start_pt.y << "]"
      << "\n\t y_coeff: [" << line->y_coeff[0] << " " << line->y_coeff[1] << " " << line->y_coeff[2] << " " << line->y_coeff[3] << "]"
      << "\n\t color: " << line->color
      << "\n\t marking: " << line->marking
      << "\n\t parsing_conf: " << line->parsing_conf
      << "\n\t rmse: " << line->rmse;
    };

  auto out = gohigh::Lines();
  memset(&out, 0, sizeof(out));
  out.dtlc = lines->dtlc();
  out.ttlc = lines->ttlc();
  out.timestamp = static_cast<uint32>(lines->header().time_stamp());
  out.left.valid = out.right.valid = out.left_left.valid = out.right_right.valid = false;
  auto * lin = &out.left;
  for (auto& line : lines->lines())
  {
      if (line.type() != line_v2::LineType_LaneLine)
      {
          LOG(INFO) << "This line type (" << line.type() << ")is not LineType_LaneLine(2), skip extact!!";
          LOG(INFO) << "start_pt: [" << line.lines_3d(0).start_pt().x() << " " << line.lines_3d(0).start_pt().y() << "]";
      }
      switch (line.position())
      {
      case line_v2::LinePosition_Left:
          lin = &out.left;
          LOG(INFO) << "extrac left lane info from J3:";
          break;
      case line_v2::LinePosition_LeftLeft:
          lin = &out.left_left;
          LOG(INFO) << "extrac left_left lane info from J3:";
          break;
      case line_v2::LinePosition_Right:
          lin = &out.right;
          LOG(INFO) << "extrac right lane info from J3:";
          break;
      case line_v2::LinePosition_RightRight:
          lin = &out.right_right;
          LOG(INFO) << "extrac right_right lane info from J3:";
          break;
      default:
          continue;
      }
      lin->id = line.id();

      auto iter = lifetime.find(lin->id);
      if (iter != lifetime.end()) {
        lin->life_time = iter->second;
      }
      else {
        LOG(ERROR) << "no life time found for id: " << lin->id;
      }
      // lin->life_time = line.life_time();
      lin->type = line.type();
      lin->conf = line.conf();
      auto& cl = line.lines_3d(0);
      lin->width = cl.width();
      lin->start_pt.x = static_cast<float>(cl.start_pt().x());
      lin->start_pt.y = static_cast<float>(cl.start_pt().y());
      lin->y_coeff[0] = static_cast<float>(cl.y_coeff(0));
      lin->y_coeff[1] = static_cast<float>(cl.y_coeff(1));
      lin->y_coeff[2] = static_cast<float>(cl.y_coeff(2));
      lin->y_coeff[3] = static_cast<float>(cl.y_coeff(3));
      lin->t_max = static_cast<float>(cl.t_max());
      lin->color = cl.color();
      lin->marking = cl.marking();
      lin->parsing_conf = cl.parsing_conf();
      lin->rmse = cl.rmse();
      lin->valid = true;
      print(lin);
  }
  lifetime.clear();
  // LOG(INFO) << "Send lanes";
  CDD_FUSION_EVENT_QUEUE.push({MsgType::CAMERA, reinterpret_cast<char*>(&out), static_cast<uint16_t>(sizeof(out))});
}

// Obstacles Parsing
static const std::string ObstacleTypeString(::google::protobuf::int32 type) {
  switch (type) {
    case ObstacleProto::ObstacleType_VehicleRear:
      return "VehicleRear";
    case ObstacleProto::ObstacleType_VehicleFull:
      return "VehicleFull";
    case ObstacleProto::ObstacleType_Pedestrian:
      return "Pedestrian";
    case ObstacleProto::ObstacleType_TrafficSign:
      return "TrafficSign";
    case ObstacleProto::ObstacleType_TrafficLight:
      return "TrafficLight";
    case ObstacleProto::ObstacleType_ParkingSlot:
      return "ParkingSlot";
    case ObstacleProto::ObstacleType_RoadSign:
      return "RoadSign";
    case ObstacleProto::ObstacleType_CrosswalkLine:
      return "CrosswalkLine";
    case ObstacleProto::ObstacleType_TrafficArrow:
      return "TrafficArrow";
    case ObstacleProto::ObstacleType_TrafficCone:
      return "TrafficCone";
    case ObstacleProto::ObstacleType_Barrel:
      return "Barrel";
    case ObstacleProto::ObstacleType_YieldMark:
      return "YieldMark";
    case ObstacleProto::ObstacleType_SpeedMark:
      return "SpeedMark";
    case ObstacleProto::ObstacleType_Character:
      return "Character";
    case ObstacleProto::ObstacleType_StopLine:
      return "StopLine";
    case ObstacleProto::ObstacleType_Diamond:
      return "Diamond";
    case ObstacleProto::ObstacleType_BicycleSign:
      return "BicycleSign";
    case ObstacleProto::ObstacleType_SpeedBumps:
      return "SpeedBumps";
    case ObstacleProto::ObstacleType_Cyclist:
      return "Cyclist";
    case ObstacleProto::ObstacleType_ParkingLock:
      return "ParkingLock";
    case ObstacleProto::ObstacleType_SpeedCamera:
      return "SpeedCamera";
    case ObstacleProto::ObstacleType_TrafficLightLens:
      return "TrafficLightLens";
    case ObstacleProto::ObstacleType_Animal:
      return "Animal";
    case ObstacleProto::ObstacleType_WarningTriangle:
      return "WarningTriangle";
    case ObstacleProto::ObstacleType_Fence:
      return "Fence";
    case ObstacleProto::ObstacleType_Unknown:
      return "Unknown";
    default:
      return "Unrecognized";
  }
}

static void ParseObstacle(const ObstacleProto::Obstacle &obstacle) {
  auto type = obstacle.type();
  std::cout << "Obstacle-" << ObstacleTypeString(type) << " -> {" << std::endl;

  // ImageSpaceInfo
  if (obstacle.has_img_info()) {
    std::cout << "img_info: {" << std::endl;
    auto &img = obstacle.img_info();
    // rect
    if (img.has_rect()) {
      auto &rect = img.rect();
      std::cout << "rect: {";
      std::cout << "left: " << rect.left() << ", "
                << "rtop: " << rect.top() << ", "
                << "right: " << rect.right() << ", "
                << "bottom: " << rect.bottom() << "}, " << std::endl;
    }
    // rect_sta
    if (img.has_rect_sta()) {
      auto &rect_sta = img.rect_sta();
      std::cout << "rect_sta: {";
      std::cout << "left: " << rect_sta.left() << ", "
                << "top: " << rect_sta.top() << ", "
                << "right: " << rect_sta.right() << ", "
                << "bottom: " << rect_sta.bottom() << "}, " << std::endl;
    }
    // sigma
    std::cout << "sigma: {";
    std::cout << "width: " << img.sigma_width() << ", "
              << "height: " << img.sigma_height() << ", "
              << "x: " << img.sigma_x() << ", "
              << "y: " << img.sigma_y() << "}" << std::endl;
    std::cout << "}, " << std::endl;
  }

  // WorldSpaceInfo
  if (obstacle.has_world_info()) {
    std::cout << "world_info: {" << std::endl;
    auto &world = obstacle.world_info();
    std::cout << "yaw: " << world.yaw() << ", "
              << "velocity.vx: " << world.vel().vx() << ", "
              << "velocity.vy: " << world.vel().vy() << ", "
              << "velocity.vz: " << world.vel().vz() << ", "
              << "length: " << world.length() << ", "
              << "width: " << world.width() << ", "
              << "height: " << world.height() << ", " << std::endl;

    std::cout << "hmw: " << world.hmw() << ", "
              << "ttc: " << world.ttc() << ", "
              << "curr_lane: " << world.curr_lane() << "," << std::endl;

    std::cout << "position_obs: {";
    std::cout << "x: " << world.position_obs().x() << ", "
              << "y: " << world.position_obs().y() << ", "
              << "z: " << world.position_obs().z() << "}, " << std::endl;

    std::cout << "ettc: " << world.ettc() << ", "
              << "acc: " << world.acc() << ", "
              << "motion_state: " << world.motion_state() << "," << std::endl;

    std::cout << "velocity_abs: {";
    std::cout << "vx: " << world.vel_abs_world().vx() << ", "
              << "vy: " << world.vel_abs_world().vy() << ", "
              << "vz: " << world.vel_abs_world().vz() << "}, " << std::endl;

    std::cout << "acc_abs: {";
    std::cout << "ax: " << world.acc_abs_world().ax() << ", "
              << "ay: " << world.acc_abs_world().ay() << ", "
              << "az: " << world.acc_abs_world().az() << "}, " << std::endl;

    std::cout << "motion_category: " << world.motion_category() << ", "
              << "position_type: " << world.position_type() << ", "
              << std::endl;

    std::cout << "yaw_rate: " << world.yaw_rate() << ", "
              << "curvature: " << world.curvature() << ", "
              << "sigma_yaw: " << world.sigma_yaw() << ", " << std::endl;

    std::cout << "sigma_vel: [";
    for (auto vel : world.sigma_vel()) {
      std::cout << vel << ", ";
    }
    std::cout << "], " << std::endl;

    std::cout << "sigma_width: " << world.sigma_width() << ", "
              << "sigma_height: " << world.sigma_height() << ", "
              << "sigma_length: " << world.sigma_length() << ", " << std::endl;

    std::cout << "sigma_position: [";
    for (auto pos : world.sigma_position()) {
      std::cout << pos << ", ";
    }
    std::cout << "], " << std::endl;

    std::cout << "conf_yaw: " << world.conf_yaw() << ", "
              << "cipv: " << world.cipv() << ", "
              << "measurement_status: " << world.measurement_status() << ", "
              << "mid_angle: " << world.mid_angle() << ", " << std::endl;

    if (world.has_obj_corner_point()) {
      std::cout << "obj_corner_point: {";
      std::cout << "objCornerPoint_x: "
                << world.obj_corner_point().objcornerpoint_x() << ", "
                << "objCornerPoint_y: "
                << world.obj_corner_point().objcornerpoint_y() << ", "
                << "objDistInLane: " << world.obj_corner_point().objdistinlane()
                << ", "
                << "objCutInFlag: " << world.obj_corner_point().objcutinflag()
                << ", "
                << "objCutInLane: " << world.obj_corner_point().objcutinlane()
                << "}, " << std::endl;
    }

    if (world.has_obj_conf()) {
      std::cout << "obj_conf: {";
      std::cout << "pos_conf: " << world.obj_conf().pos_conf() << ", "
                << "yaw_conf: " << world.obj_conf().yaw_conf() << ", "
                << "speed_conf: " << world.obj_conf().speed_conf() << ", "
                << "acc_conf: " << world.obj_conf().acc_conf() << "}, "
                << std::endl;
    }

    if (world.has_acc_ref()) {
      std::cout << "acc_ref: {";
      std::cout << "ax: " << world.acc_ref().ax() << ", "
                << "ay: " << world.acc_ref().ay() << ", "
                << "ax: " << world.acc_ref().ax() << "}" << std::endl;
    }

    std::cout << "}, " << std::endl;
  }

  // GlobalSpaceInfo
  if (obstacle.has_global_info()) {
    std::cout << "global_info: {" << std::endl;
    auto &ginfo = obstacle.global_info();

    if (ginfo.has_odom_pose()) {
      std::cout << "odom_pose: {";
      std::cout << "x: " << ginfo.odom_pose().x() << ", "
                << "y: " << ginfo.odom_pose().y() << ", "
                << "z: " << ginfo.odom_pose().z() << ", "
                << "roll: " << ginfo.odom_pose().roll() << ", "
                << "pitch: " << ginfo.odom_pose().pitch() << ", "
                << "yaw: " << ginfo.odom_pose().yaw() << "}, " << std::endl;
    }

    if (ginfo.has_odom_speed()) {
      std::cout << "odom_speed: {";
      std::cout << "x: " << ginfo.odom_speed().x() << ", "
                << "y: " << ginfo.odom_speed().y() << ", "
                << "z: " << ginfo.odom_speed().z() << "}, " << std::endl;
    }

    if (ginfo.has_odom_acceleration()) {
      std::cout << "odom_acceleration: {";
      std::cout << "x: " << ginfo.odom_acceleration().x() << ", "
                << "y: " << ginfo.odom_acceleration().y() << ", "
                << "z: " << ginfo.odom_acceleration().z() << "}, " << std::endl;
    }

    if (ginfo.has_lane_speed()) {
      std::cout << "lane_speed: {";
      std::cout << "x: " << ginfo.lane_speed().x() << ", "
                << "y: " << ginfo.lane_speed().y() << ", "
                << "z: " << ginfo.lane_speed().z() << "}, " << std::endl;
    }

    if (ginfo.has_lane_acceleration()) {
      std::cout << "lane_acceleration: {";
      std::cout << "x: " << ginfo.lane_acceleration().x() << ", "
                << "y: " << ginfo.lane_acceleration().y() << ", "
                << "z: " << ginfo.lane_acceleration().z() << "}, " << std::endl;
    }

    std::cout << "odom_s: " << ginfo.odom_s() << ", "
              << "lane_s: " << ginfo.lane_s() << ", "
              << "lane_l: " << ginfo.lane_l() << ", "
              << "left_line_dist: " << ginfo.left_line_dist() << ", "
              << "right_line_dist: " << ginfo.right_line_dist() << ", "
              << "static_flag: " << ginfo.static_flag() << ", "
              << "cutin_flag: " << ginfo.cutin_flag() << ", "
              << "pre_cutin_flag: " << ginfo.pre_cutin_flag() << ", "
              << "pre_cutin_conf: " << ginfo.pre_cutin_conf() << ", "
              << std::endl;

    if (ginfo.has_shape()) {
      std::cout << "shape: {";
      std::cout << "x: " << ginfo.shape().x() << ", "
                << "y: " << ginfo.shape().y() << ", "
                << "z: " << ginfo.shape().z() << "}, " << std::endl;
    }

    // PredictedTraj
    std::cout << "predicted_trajector: [";
    for (auto &traj : ginfo.pred_trajectories()) {
      std::cout << "{probability: " << traj.probability() << ", "
                << "anchor_idx: " << traj.anchor_idx() << ", "
                << "step_time: " << traj.step_time() << ", " << std::endl;

      std::cout << "trajectory: [";
      for (auto &tp : traj.trajectory()) {
        std::cout << "{x: " << tp.trajectory().x() << ", "
                  << "y: " << tp.trajectory().y() << ", "
                  << "z: " << tp.trajectory().z() << ", " << std::endl;
        std::cout << "covariance: [";
        for (auto &cov : tp.covariance()) {
          std::cout << cov << ", ";
        }
        std::cout << "] }, " << std::endl;
      }
      std::cout << "] }, " << std::endl;
    }
    std::cout << "], " << std::endl;

    std::cout << "cipv_flag: " << ginfo.cipv_flag() << ", "
              << "valid_flag: " << ginfo.valid_flag() << ", "
              << "cut_in_line_index: " << ginfo.cut_in_line_index() << ", "
              << "cut_in_dist: " << ginfo.cut_in_dist() << std::endl;

    std::cout << "}, " << std::endl;
  }

  // prop_info
  std::cout << "prop_info.color: " << obstacle.prop_info().color() << ", "
            << std::endl;
  std::cout << "property: [";
  for (auto &prop : obstacle.property()) {
    std::cout << prop << ", ";
  }
  std::cout << "], " << std::endl;

  std::cout << "property_name: [";
  for (auto &pn : obstacle.property_name()) {
    std::cout << pn << ", ";
  }
  std::cout << "], " << std::endl;

  std::cout << "property_type: [";
  for (auto &pt : obstacle.property_type()) {
    std::cout << pt << ", ";
  }
  std::cout << "], " << std::endl;

  std::cout << "property_conf: [";
  for (auto &pc : obstacle.property_conf()) {
    std::cout << pc << ", ";
  }
  std::cout << "], " << std::endl;

  std::cout << "tracking_fmap: [";
  for (auto &fm : obstacle.tracking_fmap()) {
    std::cout << "{data: " << fm.data() << ", "
              << "type: " << fm.type() << "}, ";
  }
  std::cout << "], " << std::endl;

  // lane_relevance
  std::cout << "lane_relevance: {";
  std::cout << "relation_type: " << obstacle.lane_relevance().relation_type()
            << ", "
            << "left_type: " << obstacle.lane_relevance().left_type() << ", "
            << "right_type: " << obstacle.lane_relevance().right_type() << "}, "
            << std::endl;

  // category
  std::cout << "category: [";
  for (auto &cate : obstacle.category()) {
    std::cout << "{property_type: " << cate.property_type() << ", "
              << "property: " << cate.property() << ", "
              << "property_name: " << cate.property_name() << ", "
              << "property_conf: " << cate.property_conf() << "},  ";
  }
  std::cout << "], " << std::endl;

  std::cout << "sub_type: " << obstacle.sub_type()
            << " predict_time: " << obstacle.predict_time()
            << " predict_frame: " << obstacle.predict_frame() << std::endl;

  std::cout << "}" << std::endl;

  std::cout << std::endl;
}

void ParseObstacleMsg(std::shared_ptr<google::protobuf::Message> message,
                      const matrix_sample::BlockDese &block) {
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));

  ObstacleProto::Obstacles *obstacles =
      static_cast<ObstacleProto::Obstacles *>(message.get());

  for (auto &obs : obstacles->obstacle()) {
    ParseObstacle(obs);
  }
}

static const std::string ObstacleRawModel(::google::protobuf::int32 model) {
  switch (model) {
    case ObstacleRawProto::ObstacleRawModel_Car:
      return "ObstacleRawModel_Car";
    case ObstacleRawProto::ObstacleRawModel_FullCar:
      return "ObstacleRawModel_FullCar";
    case ObstacleRawProto::ObstacleRawModel_Ped:
      return "ObstacleRawModel_Ped";
    case ObstacleRawProto::ObstacleRawModel_Head:
      return "ObstacleRawModel_Head";
    case ObstacleRawProto::ObstacleRawModel_Rare:
      return "ObstacleRawModel_Rare";
    case ObstacleRawProto::ObstacleRawModel_TrafficSign:
      return "ObstacleRawModel_TrafficSign";
    case ObstacleRawProto::ObstacleRawModel_TrafficLight:
      return "ObstacleRawModel_TrafficLight";
    case ObstacleRawProto::ObstacleRawModel_Lane:
      return "ObstacleRawModel_Lane";
    case ObstacleRawProto::ObstacleRawModel_RoadSign:
      return "ObstacleRawModel_RoadSign";
    case ObstacleRawProto::ObstacleRawModel_TrafficArrow:
      return "ObstacleRawModel_TrafficArrow";
    case ObstacleRawProto::ObstacleRawModel_YieldMark:
      return "ObstacleRawModel_YieldMark";
    case ObstacleRawProto::ObstacleRawModel_TrafficCone:
      return "ObstacleRawModel_TrafficCone";
    case ObstacleRawProto::ObstacleRawModel_Barrel:
      return "ObstacleRawModel_Barrel";
    case ObstacleRawProto::ObstacleRawModel_SpeedMark:
      return "ObstacleRawModel_SpeedMark";
    case ObstacleRawProto::ObstacleRawModel_Cyclist:
      return "ObstacleRawModel_Cyclist";
    case ObstacleRawProto::ObstacleRawModel_ParkingLock:
      return "ObstacleRawModel_ParkingLock";
    case ObstacleRawProto::ObstacleRawModel_SpeedCamera:
      return "ObstacleRawModel_SpeedCamera";
    case ObstacleRawProto::ObstacleRawModel_TrafficLightLens:
      return "ObstacleRawModel_TrafficLightLens";
    case ObstacleRawProto::ObstacleRawModel_ParkingSlot:
      return "ObstacleRawModel_ParkingSlot";
    default:
      return "Unknown";
  }
}

static void ParseObstacleRaw(const ObstacleRawProto::ObstacleRaw &obs) {
  std::cout << ObstacleRawModel(obs.model()) << ": -> {" << std::endl;
  std::cout << "rect: {"
            << "left: " << obs.rect().left() << ", "
            << "top: " << obs.rect().top() << ", "
            << "right: " << obs.rect().right() << ", "
            << "bottom: " << obs.rect().bottom() << "}, " << std::endl;

  std::cout << "property: [";
  for (auto prop : obs.property()) {
    std::cout << prop << ", ";
  }
  std::cout << "], " << std::endl;

  std::cout << "property_name: [";
  for (auto &pn : obs.property_name()) {
    std::cout << pn << ", ";
  }
  std::cout << "], " << std::endl;

  std::cout << "property_type: [";
  for (auto pt : obs.property_type()) {
    std::cout << pt << ", ";
  }
  std::cout << "], " << std::endl;

  std::cout << "property_conf: [";
  for (auto pc : obs.property_conf()) {
    std::cout << pc << ", ";
  }
  std::cout << "], " << std::endl;

  if (obs.has_obb()) {
    std::cout << "OBBox2D: {";
    std::cout << "corner: {"
              << "x: " << obs.obb().corner().x() << ", "
              << "y: " << obs.obb().corner().y() << ", "
              << "z: " << obs.obb().corner().z() << "}, " << std::endl;

    std::cout << "axes_pts: [";
    for (auto &pt : obs.obb().axes_pts()) {
      std::cout << "{x: " << pt.x() << ", "
                << "y: " << pt.y() << ", "
                << "z: " << pt.z() << "}, ";
    }
    std::cout << "], " << std::endl;

    std::cout << "size: [";
    for (auto s : obs.obb().size()) {
      std::cout << s << ", ";
    }
    std::cout << "], " << std::endl;
  }

  std::cout << "property_bbox: [";
  for (auto &pb : obs.property_bbox()) {
    std::cout << "{type: " << pb.type() << ", "
              << "conf: " << pb.conf() << ", " << std::endl;

    std::cout << "rect: {left: " << pb.rect().left() << ", "
              << "top: " << pb.rect().top() << ", "
              << "right: " << pb.rect().right() << ", "
              << "bottom: " << pb.rect().bottom() << "}, " << std::endl;

    std::cout << "category: [";
    for (auto &cat : pb.category()) {
      std::cout << "{property_type: " << cat.property_type() << ", "
                << "property: " << cat.property() << ", "
                << "property_name: " << cat.property_name() << ", "
                << "property_conf: " << cat.property_conf() << "}, ";
    }
    std::cout << "], " << std::endl;

    std::cout << "}, ";
  }
  std::cout << "], " << std::endl;

  std::cout << "key_points: [";
  for (auto &kp : obs.key_points()) {
    std::cout << "{point: {x: " << kp.pt().x() << ", "
              << "y: " << kp.pt().y() << ", "
              << "z: " << kp.pt().z() << "}, "
              << "type: " << kp.type() << ", "
              << "conf: " << kp.conf() << "}, " << std::endl;
  }
  std::cout << "], " << std::endl;

  std::cout << "landmarking: [";
  for (auto &ldk : obs.landmarking()) {
    std::cout << "{key_points: [";
    for (auto &kp : ldk.key_points()) {
      std::cout << "{point: {x: " << kp.pt().x() << ", "
                << "y: " << kp.pt().y() << ", "
                << "z: " << kp.pt().z() << "}, "
                << "type: " << kp.type() << ", "
                << "conf: " << kp.conf() << "}, " << std::endl;
    }
    std::cout << "], " << std::endl;

    std::cout << "property_bbox: [";
    for (auto &pb : ldk.property_bbox()) {
      std::cout << "{type: " << pb.type() << ", "
                << "conf: " << pb.conf() << ", " << std::endl;

      std::cout << "rect: {left: " << pb.rect().left() << ", "
                << "top: " << pb.rect().top() << ", "
                << "right: " << pb.rect().right() << ", "
                << "bottom: " << pb.rect().bottom() << "}, " << std::endl;

      std::cout << "category: [";
      for (auto &cat : pb.category()) {
        std::cout << "{property_type: " << cat.property_type() << ", "
                  << "property: " << cat.property() << ", "
                  << "property_name: " << cat.property_name() << ", "
                  << "property_conf: " << cat.property_conf() << "}, ";
      }
      std::cout << "] }, ";
    }
    std::cout << "], " << std::endl;
    std::cout << "}, " << std::endl;
  }
  std::cout << "], " << std::endl;

  std::cout << "scale_change: [";
  for (auto &sc : obs.scale_change()) {
    std::cout << "scale: [";
    for (auto s : sc.scale()) {
      std::cout << s << ", ";
    }
    std::cout << "], ";
  }
  std::cout << "], " << std::endl;

  std::cout << "ori_regression: [";
  for (auto &ori : obs.ori_regression()) {
    std::cout << "{name: " << ori.name() << ", "
              << "conf: " << ori.conf() << ", "
              << "id: " << ori.id() << ", "
              << "type: " << ori.type() << ", "
              << "re_scale: " << ori.re_scale() << ", "
              << "value: " << ori.value() << "}, " << std::endl;
  }
  std::cout << "], " << std::endl;

  std::cout << "key_points_raw: [";
  for (auto &kp : obs.key_points_raw()) {
    std::cout << "{key_points: [";
    for (auto &kpr : kp.key_points()) {
      std::cout << "{point: {x: " << kpr.pt().x() << ", "
                << "y: " << kpr.pt().y() << ", "
                << "z: " << kpr.pt().z() << "}, "
                << "type: " << kpr.type() << ", "
                << "conf: " << kpr.conf() << "}, " << std::endl;
    }
    std::cout << "], " << std::endl;
    std::cout << "task_type: " << kp.task_type() << "}, " << std::endl;
  }
  std::cout << "], " << std::endl;

  std::cout << "kps_cls_raw: [";
  for (auto &kcr : obs.kps_cls_raw()) {
    std::cout << "{id: [";
    for (auto i : kcr.id()) {
      std::cout << i << ", ";
    }
    std::cout << "], ";

    std::cout << "conf: [";
    for (auto c : kcr.conf()) {
      std::cout << c << ", ";
    }
    std::cout << "], ";

    std::cout << "name: [";
    for (auto &n : kcr.name()) {
      std::cout << n << ", ";
    }
    std::cout << "], ";

    std::cout << "type: " << kcr.type() << "}, " << std::endl;
  }
  std::cout << "], " << std::endl;

  std::cout << "category: [";
  for (auto &cat : obs.category()) {
    std::cout << "{property_type: " << cat.property_type() << ", "
              << "property: " << cat.property() << ", "
              << "property_name: " << cat.property_name() << ", "
              << "property_conf: " << cat.property_conf() << "}, ";
  }
  std::cout << "], " << std::endl;

  std::cout << "}" << std::endl;
  std::cout << std::endl;
}

void ParseObstacleRawMsg(std::shared_ptr<google::protobuf::Message> message,
                         const matrix_sample::BlockDese &block) {
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));

  ObstacleRawProto::ObstacleRaws *obstacle_raws =
      static_cast<ObstacleRawProto::ObstacleRaws *>(message.get());

  for (auto &obs : obstacle_raws->obstacle()) {
    ParseObstacleRaw(obs);
  }
}

void ParseCanMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block) {
  if (!matrix_sample::CanDecoder::Instance().GetCanDbcStatus()) {
    return;
  }

  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
  CANProto::CANFrames *proto_msg =
      static_cast<CANProto::CANFrames *>(message.get());

  std::cout << "\nCAN -> {";
  const int can_frame_raws_size = proto_msg->can_frame_raws_size();

  for (auto i = 0; i < can_frame_raws_size; i++) {
    const ::CANProto::CANFrameRaw &can_frame_raws =
        proto_msg->can_frame_raws(i);
    std::cout << "can_frame_raws[" << i << "]: "
              << "can_id: " << can_frame_raws.can_id() << ", "
              << "can_dlc: " << can_frame_raws.can_dlc() << ", \n";
    std::cout << "data: { ";
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (auto j = 0; j < can_frame_raws.data_size(); j++) {
      std::cout << "data[" << j << "]:" << can_frame_raws.data(j);
      if (j != can_frame_raws.data_size() - 1) {
        std::cout << ", ";
      }
      data[j] = (uint8_t)can_frame_raws.data(j);
    }
    std::cout << " },";
    std::cout << std::endl
              << "time_stamp: " << can_frame_raws.time_stamp() << ", "
              << "can_type: " << can_frame_raws.can_type() << ", "
              << "can_channele: " << can_frame_raws.can_channel() << "} \n";
    int idx = can_frame_raws.can_id();
    int dlc = can_frame_raws.can_dlc();
    int64_t ts = can_frame_raws.time_stamp();
    // start parse
    if (matrix_sample::CanDecoder::Instance().GetCanDbcStatus()) {
      matrix_sample::CanDecoder::Instance().FeedInfo(idx, dlc, data, ts);
    }
  }
  CANProto::CANFrame can_frame = proto_msg->can_frame();
  for (auto i = 0; i < can_frame.ts_size(); i++) {
    std::cout << "can_frame_ts[" << i << "]: [type: " << can_frame.ts(i).type()
              << ", "
              << "time_stamp: " << can_frame.ts(i).time_stamp() << "], "
              << std::endl;
  }
  for (auto i = 0; i < can_frame.gt_size(); i++) {
    std::cout << "can_frame_gt[" << i << "]: [type: " << can_frame.gt(i).type()
              << ", "
              << "time_stamp: " << can_frame.gt(i).time_stamp() << "], "
              << std::endl;
  }
  for (auto i = 0; i < can_frame.sp_size(); i++) {
    std::cout << "can_frame_sp[" << i
              << "]: [speed: " << can_frame.sp(i).speed() << ", "
              << "time_stamp: " << can_frame.sp(i).time_stamp() << ", "
              << "veh_speed_valid: " << can_frame.sp(i).veh_speed_valid()
              << ", \n";
    for (auto j = 0; j < can_frame.sp(i).wheel_pulse_size(); j++) {
      std::cout << "wheel_pulse[" << j
                << "]: " << can_frame.sp(i).wheel_pulse(j) << ", \n";
    }
    for (auto j = 0; j < can_frame.sp(i).wheel_pulse_valid_size(); j++) {
      std::cout << "wheel_pulse_valid[" << j
                << "]: " << can_frame.sp(i).wheel_pulse_valid(j) << ", \n";
    }
    for (auto j = 0; j < can_frame.sp(i).wheel_speed_size(); j++) {
      std::cout << "wheel_speed[" << j
                << "]: " << can_frame.sp(i).wheel_speed(j) << ", \n";
    }
    for (auto j = 0; j < can_frame.sp(i).wheel_speed_valid_size(); j++) {
      std::cout << "wheel_speed_valid[" << j
                << "]: " << can_frame.sp(i).wheel_speed_valid(j) << ", \n";
    }

    for (auto j = 0; j < can_frame.sp(i).wheel_pulse_direction_size(); j++) {
      std::cout << "wheel_pulse_direction[" << j
                << "]: " << can_frame.sp(i).wheel_pulse_direction(j) << ", \n";
    }
    for (auto j = 0; j < can_frame.sp(i).wheel_pulse_direction_valid_size();
         j++) {
      std::cout << "wheel_pulse_direction_valid[" << j
                << "]: " << can_frame.sp(i).wheel_pulse_direction_valid(j)
                << std::endl;
    }
  }
  for (auto i = 0; i < can_frame.wa_size(); i++) {
    std::cout << "can_frame_wa[" << i
              << "]: [angle: " << can_frame.wa(i).angle() << ", "
              << "time_stamp: " << can_frame.wa(i).time_stamp() << "], "
              << std::endl;
  }
  for (auto i = 0; i < can_frame.imu_size(); i++) {
    std::cout << "can_frame_imu[" << i << "]: [acc: " << can_frame.imu(i).acc()
              << ", "
              << "yaw: " << can_frame.imu(i).yaw() << ", "
              << "yaw_rate: " << can_frame.imu(i).yaw_rate() << ", "
              << "time_stamp: " << can_frame.imu(i).time_stamp() << "], "
              << std::endl;
  }
  for (auto i = 0; i < can_frame.hlbeam_size(); i++) {
    std::cout << "can_frame_wa[" << i
              << "]: [high_beam: " << can_frame.hlbeam(i).high_beam() << ", "
              << "low_beam: " << can_frame.hlbeam(i).low_beam() << ", "
              << "auto_light: " << can_frame.hlbeam(i).auto_light() << ", "
              << "time_stamp: " << can_frame.hlbeam(i).time_stamp() << "], "
              << std::endl;
  }
  for (auto i = 0; i < can_frame.udlc_size(); i++) {
    std::cout << "can_frame_udlc[" << i
              << "]: [left_type: " << can_frame.udlc(i).left_type() << ", "
              << "right_type: " << can_frame.udlc(i).right_type() << ", "
              << "time_stamp: " << can_frame.udlc(i).time_stamp() << "], "
              << std::endl;
  }
}

void ParsePedResultMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block) {
    message.get()->ParsePartialFromArray(block.meta_->data(),
                                        static_cast<int>(block.meta_size_));
    ObstacleProto::Obstacles *ped_result =
        static_cast<ObstacleProto::Obstacles *>(message.get());

    gohigh::Obstacles result;
    ParseObstacles(ped_result, &result);
    CDD_FUSION_EVENT_QUEUE.push({MsgType::CAMERA, reinterpret_cast<char*>(&result), static_cast<uint16_t>(sizeof(gohigh::Obstacles))});
}

void ParseROIMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block) {
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
//   PerceptionBaseProto::ModelROI *resize_roi =
//       static_cast<PerceptionBaseProto::ModelROI *>(message.get());
}

void ParseRuntimeMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block) {
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
//   RuntimeProto::RuntimeProto *runtime =
//       static_cast<RuntimeProto::RuntimeProto *>(message.get());
}

void ParseRuntimeEnvMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block) {
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
//   RuntimeEnvProto::RuntimeEnvProto *runtime_env =
//       static_cast<RuntimeEnvProto::RuntimeEnvProto *>(message.get());
}

void ParseScanPointsMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block){
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
//   ScanPointProto::ScanPoints *scan_points =
//       static_cast<ScanPointProto::ScanPoints *>(message.get());

}

void ParseSkeletonRawsMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block){
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
//   SkeletonProto::SkeletonRaws *skeleton_raws =
//       static_cast<SkeletonProto::SkeletonRaws *>(message.get());

}

void ParseTrafficlightMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block){
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
//   ObstacleProto::Obstacles *obstacles =
//       static_cast<ObstacleProto::Obstacles *>(message.get());
}
void ParseOdometryMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block){
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
//   OdometryProto::OdometryFrame *odometry =
//       static_cast<OdometryProto::OdometryFrame *>(message.get());
}

void ParseVehicleresultMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block){
    message.get()->ParsePartialFromArray(block.meta_->data(),
                                        static_cast<int>(block.meta_size_));
    ObstacleProto::Obstacles *obstacles =
        static_cast<ObstacleProto::Obstacles *>(message.get());

    gohigh::Obstacles result;
    ParseObstacles(obstacles, &result);
    CDD_FUSION_EVENT_QUEUE.push({MsgType::CAMERA, reinterpret_cast<char*>(&result), static_cast<uint16_t>(sizeof(gohigh::Obstacles))});
}

void ParseBoxes3drawMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block){
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
  	// Box3DRawProto::Box3DRaws *box3fraws =
    //   static_cast<Box3DRawProto::Box3DRaws *>(message.get());
}

void ParseLaneresultMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block){
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
  LaneProto::Lines *lines =
    static_cast<LaneProto::Lines *>(message.get());

  lifetime.clear();
  for (auto& line : lines->lines())
  {
    lifetime.insert(std::pair<int,float>(line.id(), line.life_time()));
  }
  // memset(lifetime, 0, 4);
}
void ParseLaneparsingMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block){
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
  	// ParsingProto::Parsing *parsing =
    //   static_cast<ParsingProto::Parsing *>(message.get());
}
void ParseParsingMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block){
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
  	// ParsingProto::Parsing *parsing =
    //   static_cast<ParsingProto::Parsing *>(message.get());
}
void ParseOnlinecalibMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block){
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
  	// OnlineCalibMsgProto::OnlineCalibMsgProto *Onlinecalib =
    //   static_cast<OnlineCalibMsgProto::OnlineCalibMsgProto *>(message.get());
}
void ParseIspparamMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block){
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
  	// ISPParamProto::ISPParamProto *ispparam =
    //   static_cast<ISPParamProto::ISPParamProto *>(message.get());
}
void ParseObjectMsg(std::shared_ptr<google::protobuf::Message> message,
                 const matrix_sample::BlockDese &block){
  message.get()->ParsePartialFromArray(block.meta_->data(),
                                       static_cast<int>(block.meta_size_));
  	// object::Object *object =
    //   static_cast<object::Object *>(message.get());
}
