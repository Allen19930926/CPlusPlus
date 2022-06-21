#ifndef F58F3DBF_4029_4C5F_885B_23B0F07B1B05
#define F58F3DBF_4029_4C5F_885B_23B0F07B1B05

struct V2xAdasMsgHeader
{
    uint32_t  separator;
    uint16_t  msgId;
    uint16_t  rsv;
    uint32_t  msgLen;
    uint16_t  seqNo;
    uint16_t  crc;
    uint64_t  timeStamp;
};

#endif /* F58F3DBF_4029_4C5F_885B_23B0F07B1B05 */
