#ifndef E3DF7E56_B5EC_46DD_B5F2_D19B8AC0EB70
#define E3DF7E56_B5EC_46DD_B5F2_D19B8AC0EB70

#include <memory>
#include <vector>

struct FusionChatHeader
{
    uint32_t sensor_type;
    uint32_t time_stamp;
    uint32_t objectNum;
    uint32_t bodyLength;
};

struct FusionChatBody
{
    FusionChatBody(uint32_t length) : body(length, 0) {}
    std::vector<char> body;
};

class FusionChatMessage {
public:
    static constexpr uint32_t HeaderLength = 16;
    static constexpr uint32_t MaxBodyLength = 1024 * 10;

    FusionChatMessage()
    {
        head_ptr = std::make_shared<FusionChatHeader>();
        body_ptr = std::make_shared<FusionChatBody>(0);
    }

    FusionChatMessage(FusionChatMessage&& r_value)
    {
        FusionChatMessage();
        head_ptr.swap(r_value.head_ptr);
        body_ptr.swap(r_value.body_ptr);
    }

    char *Data() { return reinterpret_cast<char*>(head_ptr.get()); }

    char *Body() { return body_ptr->body.data(); }

    const int BodyLength() { return head_ptr->bodyLength; }

    const FusionChatHeader& Header() {return *head_ptr;}

    bool DecodeHeader()
    {
        if (head_ptr->bodyLength >= MaxBodyLength)
        {
            return false;
        }
        body_ptr = std::make_shared<FusionChatBody>(head_ptr->bodyLength);
        return true;
    }

protected:
    std::shared_ptr<FusionChatHeader> head_ptr;
    std::shared_ptr<FusionChatBody> body_ptr;
};

#endif /* E3DF7E56_B5EC_46DD_B5F2_D19B8AC0EB70 */
