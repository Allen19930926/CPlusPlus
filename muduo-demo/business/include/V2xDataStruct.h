#ifndef F58F3DBF_4029_4C5F_885B_23B0F07B1B05
#define F58F3DBF_4029_4C5F_885B_23B0F07B1B05

/* build-in struct */
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


/* reprot struct */

// EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT
struct GSentryStatus
{
    uint8_t gSentryStatus;    //0:未知1:初始化2:激活 3:正常 4:异常 5:退出 其他：保留
    bool    faultStatus;      //是否发生故障  True：有故障；False：无故障
};

// EV_GSENTRY_ADAS_CALC_MAPINFO_REPORT
struct MapAddResult
{
    uint16_t  OffsetTolink;     //本车行驶偏移道路中心线距离左负右正, 取值-2000~2001，单位0.01m
    uint16_t  OffsetTolane;     //本车行驶偏移道路中心线距离左负右正, 取值-2000~2001，单位0.01m
    uint32_t  DistToNode;       //本车距离驶向路口的距离(到停止线距离), 取值0~100001，单位0.01m
    bool      IsAtAcross;       //是否在路口（车辆中心点过停止线）True：在路口；False：不在路口
};

#endif /* F58F3DBF_4029_4C5F_885B_23B0F07B1B05 */
