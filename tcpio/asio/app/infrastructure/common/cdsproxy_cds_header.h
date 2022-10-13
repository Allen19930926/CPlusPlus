#ifndef CDSPROXY_CDS_HEADER_H_
#define CDSPROXY_CDS_HEADER_H_

#include <stdint.h>

#pragma pack(push, 1)
typedef struct MsgHead
{
    
    uint8_t head0;     ///< 0x00
    uint8_t head1;     ///< 0xff
    uint8_t version;   ///< 0 : msg struct version
    uint8_t msgtype;   ///< 0 : control data 1 : data data 2 : heart data
    uint8_t format;    ///< 0 : json and non_base64 1 : json and base64 other : reserve
    uint8_t rsv1;
    uint8_t rsv2;
    uint8_t rsv3;

    uint64_t timestamp;  ///< UTC(us)
    uint32_t msgid;      ///< 0x01 can msg
    uint32_t matchid;    ///< 

    uint32_t datalen;   ///< big-endian, exclude head

}MsgHead_t;

#pragma pack(pop)

#endif

