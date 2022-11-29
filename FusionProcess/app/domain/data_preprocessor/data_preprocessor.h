#ifndef A3C34B88_77AF_4DC8_86F9_75D197A17265
#define A3C34B88_77AF_4DC8_86F9_75D197A17265

#include "infrastructure/common/fusion_chat_message.h"

#include <vector>

class DataPreprocessor
{
public:
    DataPreprocessor();
    void ProcessReadMsg(FusionChatMessage msg);

private:
    using ProcessFunc = void(DataPreprocessor::*)(char*, uint32_t);
    void ProcessV2xMsg(char* data, uint32_t len);
    void ProcessCameraMsg(char* data, uint32_t len);

private:
    std::vector<ProcessFunc> func_table;
};

#endif /* A3C34B88_77AF_4DC8_86F9_75D197A17265 */
