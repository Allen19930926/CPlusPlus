// Copyright 2020 Horizon Robotics.
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include "common/data_handler.h"
#include "common/define.h"
#include "communication/subscriber.h"
#include "hobot-adas-sdk.h"
#include "hobot-auto/proto_factory.h"
#include "nlohmann/json.hpp"
#include "protocol/frame.v2.pb.h"
#ifdef ADAS_JOURNEY
#include "spi_service_protocol.h"
#endif
namespace HobotADAS {
using FrameProtoMsg = hobot::message::ProtoMsg<FrameV2Proto::Frame>;
using FrameProtoSerializer =
    hobot::message::ProtobufSerializer<FrameV2Proto::Frame>;
using SubPtr =
    std::shared_ptr<hobot::communication::Subscriber<FrameProtoSerializer>>;
using CallBack =
    std::function<void(std::shared_ptr<google::protobuf::Message>)>;
using ListCallBack = std::list<CallBack>;
struct Framework::Impl {
  Impl()
      : proto_req_(0xFFFFFFFF)
#ifdef ADAS_JOURNEY
        ,
        pub_can_input_(new PubCANInput("/tmp/can_input.ipc"))
#endif
  {
  }
  ~Impl() {}
  bool Init(std::string config_file_path) {
    config_file_path_ = config_file_path;
    try {
      // parse config
      std::ifstream cfg_file(config_file_path_);

      if (!cfg_file.is_open()) {
        std::cout << "open config file " << config_file_path_ << " failed!"
                  << std::endl;
        return false;
      }

      const nlohmann::json &json = nlohmann::json::parse(cfg_file);

      if (json.is_null()) {
        std::cout << "parse config file " << config_file_path_ << " failed!"
                  << std::endl;
        return false;
      }

      // init communication
      auto ret_code = hobot::communication::Init(config_file_path_);

      if (ret_code != hobot::communication::COMM_CODE_OK) {
        std::cout << "init communication failed: " << ErrorMsg(ret_code)
                  << std::endl;
        return false;
      }

      for (auto &&node : json.at("participants")) {
        std::string protocol;
        int id = -1;
        node.at("id").get_to(id);
        node.at("protocol").get_to(protocol);

        if (id != -1 && !protocol.empty()) {
          partipants_config_[protocol].push_back(id);
        }
      }
    } catch (...) {
      std::cout << "parse config json failed!" << std::endl;
      return false;
    }

    if (partipants_config_.empty()) {
      std::cout << "partipants is empty!" << std::endl;
      return false;
    }
    return true;
  }
  void Start() {
    for (auto &it : function_maps_) {
      std::cout << "RegisterFunction topic name : " << it.first << std::endl;
    }
    // one subscriber deal one protocol
    for (auto it = partipants_config_.begin(); it != partipants_config_.end();
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

      auto subscriber =
          hobot::communication::Subscriber<FrameProtoSerializer>::NewExt(
              comm_attr, topic, hobot::communication::DUMB_DOMAIN_ID,
              std::bind(&Framework::Impl::Process, this, std::placeholders::_1),
              protocol_type);

      if (!subscriber) {
        std::cout << "create subscriber failed" << std::endl;
        return;
      }

      subscribers_.push_back(subscriber);
    }
  }
  void Terminate() {
    // HobotADAS::ZmqNodeManager::Instance().CloseNode(0);
  }
  bool RegisterFunction(const std::string &topic, CallBack callback) {
    auto iter = function_maps_.find(topic);
    if (iter == function_maps_.end()) {
      ListCallBack callback_list;
      callback_list.push_back(callback);
      auto callback_pair = std::make_pair(topic, callback_list);
      function_maps_.insert(callback_pair);
    } else {
      iter->second.push_back(callback);
    }

    return true;
  }
#ifdef ADAS_JOURNEY
  void FeedCAN(CANInput &&can_input) {
    int can_frame_num = can_input.can_frames.size();
    pub_can_input_->IpcCANInputPub(
        reinterpret_cast<uint8_t *>(&can_input.header),
        sizeof(can_input.header),
        reinterpret_cast<uint8_t *>(&can_input.can_frames[0]),
        can_frame_num * sizeof(can_frame));
  }
#endif
  void Process(const std::shared_ptr<FrameProtoMsg> &frame_msg) {
    if (frame_msg == nullptr) {
      std::cout << "the message is null" << std::endl;
      return;
    }
    std::vector<uint8_t> meta;
    std::vector<uint8_t *> data;
    std::vector<int> data_size;
    // parse frame
    matrix_sample::DataHandler::Instance().CreateBlockData(frame_msg, &meta,
                                                           &data, &data_size);
    if (data.size() > 0 && data_size.size() > 0) {
      std::unique_ptr<void, std::function<void(void *)>> frame(
          CreatePackFrameFromDatas(static_cast<int>(data.size()),
                                   (const uint8_t **)data.data(),
                                   data_size.data()),
          &DestroyPackFrame);
      if (frame != nullptr) {
        // create dese
        std::vector<matrix_sample::FrameDese> dese_vec;
        matrix_sample::DataHandler::Instance().HandleFrame(frame.get(),
                                                           &dese_vec);
        // for all topic data
        for (auto iter = dese_vec.begin(); iter != dese_vec.end(); ++iter) {
          auto func_set = function_maps_.find(iter->topic);
          if (func_set == function_maps_.end()) continue;
          // for all reg function in list
          for (auto &reg_func : func_set->second) {
            // for all data in parsed block data
            for (auto &iter_block : iter->block) {
              auto msg = factory_.MakeProtobufMsg(
                  (iter->topic).substr(0, (iter->topic).rfind('#')));
              if (msg != nullptr) {
                msg.get()->ParsePartialFromArray(
                    iter_block.meta_->data(),
                    static_cast<int>(iter_block.meta_size_));
                reg_func(msg);
              }
            }
          }
        }
      }
    }
  };

 private:
  std::string config_file_path_;
  bool running_;
  std::map<std::string, ListCallBack> function_maps_;
  // DataReceiver data_receiver_;
#ifdef ADAS_JOURNEY
  std::unique_ptr<PubCANInput> pub_can_input_;
#endif
  uint32_t proto_req_;
  std::vector<SubPtr> subscribers_;
  std::map<std::string, std::vector<int>> partipants_config_;
  hobot_auto::ProtoFactory factory_;
};

Framework::Framework() : impl_{new Framework::Impl{}} {}
Framework::~Framework() {}

Framework &Framework::Instance() {
  static Framework obj;
  return obj;
}

bool Framework::RegisterFunction(const std::string &topic, CallBack callback) {
  return impl_->RegisterFunction(topic, callback);
}
bool Framework::Init(std::string config_file_path) {
  return impl_->Init(config_file_path);
}
#ifdef ADAS_JOURNEY
void Framework::FeedCAN(CANInput &&can_input) {
  impl_->FeedCAN(std::move(can_input));
}
#endif
void Framework::Start() { impl_->Start(); }
void Framework::Terminate() { impl_->Terminate(); }

}  // namespace HobotADAS
