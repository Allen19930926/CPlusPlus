// Copyright 2020 Horizon Robotics.

#include "common/data_handler.h"

namespace matrix_sample {
void DataHandler::CreateBlockData(
    const std::shared_ptr<hobot::message::ProtoMsg<FrameV2Proto::Frame>>&
        frame_msg,
    std::vector<uint8_t>* meta, std::vector<uint8_t*>* data,
    std::vector<int>* data_size) {
  int metaSize = frame_msg->proto.ByteSize();

  if (metaSize > 0) {
    meta->resize(metaSize);
    bool isOk = frame_msg->proto.SerializeToArray(meta->data(), metaSize);

    if (isOk) {
      data->push_back(meta->data());
      data_size->push_back(metaSize);

      auto data_vec = frame_msg->GetData().get()->GetDataVec();
      const auto& data_size_vec = frame_msg->GetData().get()->GetDataSizeVec();

      if (data_vec.size() == data_size_vec.size()) {
        for (int i = 0; i < data_vec.size(); i++) {
          data->push_back(static_cast<uint8_t*>(data_vec[i]));
          data_size->push_back(data_size_vec[i]);
        }
      }
    }
  }
}

void DataHandler::HandleFrame(const PackFrame& frame,
                              std::vector<FrameDese>* dese) {
  int topics_count = GetTopicsCount(frame);
  char** topic_list = GetTopicList(frame);

  std::set<std::string> topic_set;
  Deduplication(topic_list, topics_count, &topic_set);
  FreeTopicList(topic_list);

  for (auto topic_it = topic_set.begin(); topic_it != topic_set.end();
       topic_it++) {
    std::string message_name = (*topic_it).substr(0, (*topic_it).rfind('#'));

    int block_count = GetTopicBlockCount(frame, message_name.c_str(), 0);

    SingleBlock* block = new SingleBlock[block_count];
    GetMetaWithData(frame, message_name.c_str(), 0, block_count, &block);

    FrameDese frame_dese;
    frame_dese.topic = (*topic_it);

    for (int i = 0; i < block_count; i++) {
      frame_dese.block.push_back(
          (BlockDese{block[i].gen_ts, block[i].done_ts, block[i].meta,
                     block[i].meta_size, block[i].data, block[i].data_size}));
    }

    dese->push_back(frame_dese);
    delete[] block;
  }
}

void DataHandler::Deduplication(char** list, int size,
                                std::set<std::string>* set) {
  for (int i = 0; i < size; i++) {
    set->insert(*(list + i));
  }
}
}  // namespace matrix_sample
