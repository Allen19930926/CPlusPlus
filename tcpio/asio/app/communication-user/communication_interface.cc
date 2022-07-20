// Copyright 2020 Horizon Robotics.
// Created by hongliang.zhai on 16/12/2021

/**
 * @file main.cc
 * @brief sample of frame receive and parse.
 */

#include "communication_interface.h"
#include <iostream>
#include <fstream>
#include "can_decoder.h"
#include "common/data_handler.h"
#include "common/define.h"
#include "msg_user.h"
#include "communication/subscriber.h"
#include "nlohmann/json.hpp"
#include "protocol/frame.v2.pb.h"

using FrameProtoMsg = hobot::message::ProtoMsg<FrameV2Proto::Frame>;
using FrameProtoSerializer =
    hobot::message::ProtobufSerializer<FrameV2Proto::Frame>;
std::vector<
    std::shared_ptr<hobot::communication::Subscriber<FrameProtoSerializer>>>
    subscribers;

matrix_sample::MsgUser user;
const std::string config_file = "communication.json";

int start_j3_communicaiton() {

    // if (matrix_sample::util::GenerateFile(config_file,
    //                                       matrix_sample::conn_content) == 0) {
    //   std::cout << "generate config file " << config_file << " failed!"
    //             << std::endl;
    //   return -1;
    // }
  
  std::map<std::string, std::vector<int>> partipants_config;

  try {
    // parse config
    std::ifstream cfg_file(config_file);

    if (!cfg_file.is_open()) {
      std::cout << "open config file " << config_file << " failed!"
                << std::endl;
      return -1;
    }

    const nlohmann::json &json = nlohmann::json::parse(cfg_file);

    if (json.is_null()) {
      std::cout << "parse config file " << config_file << " failed!"
                << std::endl;
      return -1;
    }

    // init communication
    auto ret_code = hobot::communication::Init(config_file);

    if (ret_code != hobot::communication::COMM_CODE_OK) {
      std::cout << "init communication failed: " << ErrorMsg(ret_code)
                << std::endl;
      return -1;
    }

    for (auto &&node : json.at("participants")) {
      std::string protocol;
      int id = -1;
      node.at("id").get_to(id);
      node.at("protocol").get_to(protocol);

      if (id != -1 && !protocol.empty()) {
        partipants_config[protocol].push_back(id);
      }
    }
  } catch (...) {
    std::cout << "parse config json failed!" << std::endl;
    return -1;
  }

  if (partipants_config.empty()) {
    std::cout << "partipants is empty!" << std::endl;
    return -1;
  }


  // one subscriber deal one protocol
  for (auto it = partipants_config.begin(); it != partipants_config.end();
       it++) {
    std::string topic = "*";
    hobot::communication::ProtocolType protocol_type =
        hobot::communication::PROTOCOL_HYBRID;
    hobot::communication::CommAttr comm_attr;

    if (it->first == "zmq_ipc") {
      protocol_type = hobot::communication::PROTOCOL_ZMQ_IPC;
    } else if (it->first == "zmq_tcp") {
      protocol_type = hobot::communication::PROTOCOL_ZMQ_TCP;
    } else if (it->first == "zmq_epgm") {
      protocol_type = hobot::communication::PROTOCOL_ZMQ_EPGM;
    } else if (it->first == "sdio") {
      protocol_type = hobot::communication::PROTOCOL_SDIO;
    } else if (it->first == "pcie") {
      protocol_type = hobot::communication::PROTOCOL_PCIE;
    }

    for (auto id : it->second) {
      comm_attr.participant_attrs_.push_back(
          hobot::communication::ParticipantAttr{id});
    }

    if (comm_attr.participant_attrs_.empty()) {
      continue;
    }

    auto subscriber =//创建事件订阅器，最终存储到全局变量数组subscribers中。
                     //回调函数将订阅消息内容解析为多（3）维数组，并推送到msguser的消息队列
        hobot::communication::Subscriber<FrameProtoSerializer>::NewExt(
            comm_attr, topic, hobot::communication::DUMB_DOMAIN_ID,
            [protocol_type](
                const std::shared_ptr<FrameProtoMsg> &frame_msg,
                const std::string topic) {
              // std::cout << "------------------------------" << std::endl
              //           << "receive topic(" << protocol_type << "): " << topic
              //           << std::endl
              //           << "------------------------------" << std::endl;

              std::vector<uint8_t> meta;

              std::vector<uint8_t *> data;
              std::vector<int> data_size;

              // parse frame
              matrix_sample::DataHandler::Instance().CreateBlockData(
                  frame_msg, &meta, &data, &data_size);

              if (data.size() > 0 && data_size.size() > 0) {
                std::unique_ptr<void, std::function<void(void *)>> frame(
                    CreatePackFrameFromDatas(static_cast<int>(data.size()),
                                             (const uint8_t **)data.data(),
                                             data_size.data()),
                    &DestroyPackFrame);

                if (frame != nullptr) {
                  // create dese
                  std::vector<matrix_sample::FrameDese> dese_vec;
                  matrix_sample::DataHandler::Instance().HandleFrame(
                      frame.get(), &dese_vec);
                  if (dese_vec.size() > 0) {
                    user.RecvMsg(&dese_vec);
                  }
                }
              }
            },
            protocol_type);

    if (!subscriber) {
      std::cout << "create subscriber failed" << std::endl;
      return -1;
    }

    subscribers.push_back(subscriber);
  }

//   handle frame
  user.HandleMsg();   //生产-消费者模式，拉起消息处理线程
}

