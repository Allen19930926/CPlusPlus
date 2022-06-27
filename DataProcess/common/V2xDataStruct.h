#ifndef F58F3DBF_4029_4C5F_885B_23B0F07B1B05
#define F58F3DBF_4029_4C5F_885B_23B0F07B1B05

#define ADAS_SPAT_INFO_NUM              5
#define ROAD_POINTS_NUM_PER_LANE        31
#define CONNECTION_NUM_PER_LANE         4
#define CONNECTION_NUM_PER_LINK         4

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

struct VehicleSize
{
    uint16_t length;            /* 目标车身长度,取值0~4095，单位0.01m */
    uint16_t width;             /* 目标车身长度,取值0~1023，单位0.01m */
    uint8_t  height;            /* 目标车身长度,取值0~127，单位0.05m */
};

struct VehicleBrakes
{
    uint8_t brakePedalStatus;       /* 指示刹车踏板状态，是否处在被踩下状态。0 Unavailable：系统不可用。1 Off：制动踏板未踩下。2 On：系制动踏板被踩下。 */
    uint8_t absBrakes;              /* 定义刹车防抱死系统（ABS）状态。0 Unavailable：系统未装备或不可用。1 Off：系统处于关闭状态。*/
    uint8_t brakeAppliedStatus;     /*定义四轮分别的刹车状态,当车辆进行刹车时，该数值分别指示了四组轮胎的刹车情况。
                                    0 unavailable：未知
                                    1 leftFront:左前轮刹车激活
                                    2 leftRear：左后轮刹车激活
                                    3 rightFront：右前轮刹车激活
                                    4 rightRear：右后轮刹车激活 */
    uint8_t brakeBoostApplied;      /* 通过刹车辅助系统的状态，指示车辆紧急刹车状态。
                                    0 Unavailable：系统未装备或不可用。
                                    1 Off：系统处于关闭状态。
                                    2 On：系统处于开启状态 */
    uint8_t auxiliaryBrakeStatus;   /* 指示刹车辅助系统状态（通常为手刹）
                                    0 Unavailable：系统未装备或不可用。
                                    1 Off：系统处于关闭状态。
                                    2 On：系统处于开启状态
                                    3 reserved：预留 */
};

struct VehicleAccelSet
{
    int16_t longitude;          /* 纵向加速度。向前加速为正，反向为负。取值-5000~5000，单位0.01m/s2 */
    int16_t latitude;           /* 横向加速度。向右加速为正，反向为负。取值-5000~5000，单位0.01m/s2 */
};

struct Position
{
    int32_t latitude;           /* 定义纬度数值，北纬为正，南纬为负。取值-900000000~900000001，单位1e-7° */
    int32_t longitude;          /* 定义经度数值。东经为正，西经为负。取值-1799999999~1800000001，单位1e-7° */
    int32_t elevation;          /* 车辆海拔，-4096~61439，无效值-4096，单位0.1m */
};

struct RoadPoint
{
    int32_t latitude;           /* 定义纬度数值，北纬为正，南纬为负。取值-900000000~900000001，单位1e-7° */
    int32_t longitude;          /* 定义经度数值。东经为正，西经为负。取值-1799999999~1800000001，单位1e-7° */
    int32_t elevation;          /* 海拔，-4096~61439，无效值-4096，单位0.1m */
};

struct ObjMapAddResult
{
    uint16_t  offsetTolink;     /* 本车行驶偏移道路中心线距离左负右正, 取值-2000~2001，单位0.01m */
    uint16_t  offsetTolane;     /* 本车行驶偏移道路中心线距离左负右正, 取值-2000~2001，单位0.01m */
    uint32_t  distToNode;       /* 本车距离驶向路口的距离(到停止线距离), 取值0~100001，单位0.01m */
    bool      isAtAcross;       /* 是否在路口（车辆中心点过停止线）True：在路口；False：不在路口 */
    int8_t    linkRealtion;     /* 远车所在车道相对本车的道路关系（一个路口内）,取值-1~7，无效值-1
                                0:SameLink 相同车道
                                1:DiffNodeRightLink不同驶向节点右车道
                                2:SameNodeRightLink相同驶向节点右车道
                                3:DiffNodeFrontLink不同驶向节点前方车道
                                4:SameNodeFrontLink相同驶向节点前方车道
                                5:DiffNodeLeftLink不同驶向节点左车道
                                6:SameNodeLeftLink相同驶向节点左车道
                                7:OppositeLink对向车道 */
};

struct RegulatorySpeedLimit
{
    uint16_t minSpeed;          /* 道路车速下限,取值0~8191,无效值8191,单位 0.02 m/s */
    uint16_t maxSpeed;          /* 道路车速上限,取值0~8191,无效值8191,单位 0.02 m/s */
};

struct UpstreamNode
{
    int32_t  RegionId;          /* 定义地图中各个划分区域的ID号,-1~65535,数值0仅用于测试，-1表示无效*/
    uint16_t id;                /* 上游节点ID。一个节点的ID在同一个区域内是唯一的。0~65535,数值0～255预留 */
};

struct ConnectsToLaneIdList
{
    uint16_t belongLinkID;      /* 所属道路ID，与downLinksList的id号保持一致,0~65535,数值0～255预留 */
    uint8_t  laneID;            /* 所属车道ID，0～255,无效值0,255预留 */
};

struct CurLink
{
    UpstreamNode            upstreamNode;
    uint16_t                downNode;       /* 当前车辆驶向节点ID,0~65535,数值0～255预留 */
    uint16_t                linkWidth;      /* 定义车道宽度,0~32767,单位cm */
    RoadPoint               roadPoints[ROAD_POINTS_NUM_PER_LANE];
                                            /*定义一个有向路段上的中间位置点列表，用来表达路段上截面或片段的特殊属性。
                                            列表中所有位置点需按上游至下游顺序排列。 */
    RegulatorySpeedLimit    speedLimit;     /* 当前道路限速信息 */
};

struct CurLane
{
    uint8_t                 laneID;         /* 车道LaneID,车道定义在每一条有向路段上，同一条有向路段上的每个车道，都拥有一个单独的ID。
                                            车道号，以该车道行驶方向为参考，自左向右从1开始编号。0～255,无效值0,255预留 */
    uint16_t                laneWidth;      /* 定义车道宽度,0~32767,单位cm */
    uint16_t                maneuvers;      /* 本车道可进行转向的行为，定义为bitmap，各bit表示允许的转向行为
                                            bit0:maneuverStraightAllowed 允许直行 true：是 false：否
                                            bit1:maneuverLeftAllowed 允许左转 true：是 false：否
                                            bit2:maneuverRightAllowed 允许右转 true：是 false：否
                                            bit3:maneuverUTurnAllowed 允许掉头 true：是 false：否
                                            bit4:maneuverLeftTurnOnRedAllowed 红灯允许左转 true：是 false：否
                                            bit5:maneuverRightTurnOnRedAllowed 红灯允许右转 true：是 false：否
                                            bit6:maneuverLaneChangeAllowed 允许改变到外侧车道 true：是 false：否
                                            bit7:maneuverNoStoppingAllowed 不允许停止线停车 true：是 false：否
                                            bit8:yieldAllwaysRequired 上述允许的动作不受保护（黄灯） true：是 false：否
                                            bit9:goWithHalt 停车后继续 true：是 false：否
                                            bit10:caution 小心驶过停止线 true：是 false：否
                                            bit11:reserved1 预留 */
    ConnectsToLaneIdList    connectsToLaneIdList[CONNECTION_NUM_PER_LANE];
                                            /* 定义路段中每条车道，在下游路口处与下游路段中车道的转向连接关系列表。
                                            connectingLane用于定位上游车道转向连接的下游车道。
                                            包括下游车道ID以及该转向的允许行驶行为，下游车道ID的作用范围是该车道所在的路段。 */
    RoadPoint               roadPoints[ROAD_POINTS_NUM_PER_LANE];
                                            /*定义一个有向路段上的中间位置点列表，用来表达路段上截面或片段的特殊属性。
                                            列表中所有位置点需按上游至下游顺序排列。 */
    uint8_t                 lanePointNextNo;/* 定义车辆将要行驶的下一个点为points的第几个点,1~32 */
};

struct DownLanesList
{
    uint8_t                 id;         /* 下游对应车道id,0～255,无效值0,255预留 */
    uint16_t                linkWidth;  /* 定义车道宽度,0~32767,单位cm */
    RoadPoint               roadPoints[ROAD_POINTS_NUM_PER_LANE];
                                        /*定义一个有向路段上的中间位置点列表，用来表达路段上截面或片段的特殊属性。
                                        列表中所有位置点需按上游至下游顺序排列。 */
};

struct DownLinksList
{
    uint8_t                 id;         /* 道路ID（为快速匹配远车belongLinkID，该ID号自定义，不采用标准的上游和下游节点号）,0～255 */
    uint16_t                linkWidth;  /* 定义车道宽度,0~32767,单位cm */
    RoadPoint               roadPoints[ROAD_POINTS_NUM_PER_LANE];
                                        /*定义一个有向路段上的中间位置点列表，用来表达路段上截面或片段的特殊属性。
                                        列表中所有位置点需按上游至下游顺序排列。 */
    DownLanesList           downLanesList[4];   /* 定义下游对应车道列表 */
    uint8_t                 PhaseID;    /* 从上游切换至本Link需要查看的PhaseID,0~255,无效值0 */


// todo, undefined
    uint8_t                 Azimuth;
};

struct AdjacentLane
{
    uint8_t     laneID;                     /* 车道LaneID,车道定义在每一条有向路段上，同一条有向路段上的每个车道，都拥有一个单独的ID。
                                            车道号，以该车道行驶方向为参考，自左向右从1开始编号。0～255,无效值0,255预留 */
    uint16_t    laneWidth;                  /* 定义车道宽度,0~32767,单位cm */
    uint16_t    maneuvers;                  /* 本车道可进行转向的行为，定义为bitmap，各bit表示允许的转向行为
                                            bit0:maneuverStraightAllowed 允许直行 true：是 false：否
                                            bit1:maneuverLeftAllowed 允许左转 true：是 false：否
                                            bit2:maneuverRightAllowed 允许右转 true：是 false：否
                                            bit3:maneuverUTurnAllowed 允许掉头 true：是 false：否
                                            bit4:maneuverLeftTurnOnRedAllowed 红灯允许左转 true：是 false：否
                                            bit5:maneuverRightTurnOnRedAllowed 红灯允许右转 true：是 false：否
                                            bit6:maneuverLaneChangeAllowed 允许改变到外侧车道 true：是 false：否
                                            bit7:maneuverNoStoppingAllowed 不允许停止线停车 true：是 false：否
                                            bit8:yieldAllwaysRequired 上述允许的动作不受保护（黄灯） true：是 false：否
                                            bit9:goWithHalt 停车后继续 true：是 false：否
                                            bit10:caution 小心驶过停止线 true：是 false：否
                                            bit11:reserved1 预留 */
    RoadPoint   roadPoints[ROAD_POINTS_NUM_PER_LANE];
                                            /*定义一个有向路段上的中间位置点列表，用来表达路段上截面或片段的特殊属性。
                                            列表中所有位置点需按上游至下游顺序排列。 */
};

struct BelongsLink
{
    uint16_t    upstreamNodeId; /*  */
    uint16_t    downNodeId;
    uint8_t     laneID;
};



/***************************************** reprot struct *****************************************/

/* EV_GSENTRY_ADAS_PROCESS_STATUS_REPORT */
struct GSentryStatus
{
    uint8_t gSentryStatus;      /* 0:未知 1:初始化 2:激活 3:正常 4:异常 5:退出 其他：保留 */
    bool    faultStatus;        /* 是否发生故障  True：有故障；False：无故障 */
};

/* EV_GSENTRY_ADAS_CALC_MAPINFO_REPORT */
struct MapAddResult
{
    uint16_t  offsetTolink;     /* 本车行驶偏移道路中心线距离左负右正, 取值-2000~2001，单位0.01m */
    uint16_t  offsetTolane;     /* 本车行驶偏移道路中心线距离左负右正, 取值-2000~2001，单位0.01m */
    uint32_t  distToNode;       /* 本车距离驶向路口的距离(到停止线距离), 取值0~100001，单位0.01m */
    bool      isAtAcross;       /* 是否在路口（车辆中心点过停止线）True：在路口；False：不在路口 */
};

/* EV_GSENTRY_ADAS_SPATINFO_REPORT */
struct AdasSpatInfo
{
    uint64_t    timeStamp;      /* 时间戳, 取值0~2^64-1 */
    bool        spatInfoValid;  /* 交通灯信息是否有效 */
    int32_t     belongsNodeId;  /* 信号灯信息所属node id, 取值0~65535,数值0～255预留为测试使用 */
    uint8_t     phaseID;        /* 信号灯相位ID，取值0~255，无效值0 */
    uint8_t     lightState;     /* 当前信号灯相位的灯色状态 ,取值0~8
                                0：unavailable 未知
                                1：dark 熄灭
                                2：flashing-red红灯闪烁
                                3：red红灯亮
                                4：flashing-green绿灯闪烁
                                5：permissive-green通行允许（可能存在冲突车流）
                                6：protected-green通行保护（继续前进）
                                7：yellow黄灯亮
                                8：flashing-yellow黄灯闪烁（警示灯）*/
    uint16_t    curCoutingTime; /* 当前路口灯色剩余时间  红变绿剩余时间/绿变红剩余时间，0~36001 无效值36001，单位0.1s */
    uint8_t     nextLight;      /* 当前相位下一个灯色 ,取值0~8
                                0：unavailable 未知
                                1：dark 熄灭
                                2：flashing-red红灯闪烁
                                3：red红灯亮
                                4：flashing-green绿灯闪烁
                                5：permissive-green通行允许（可能存在冲突车流）
                                6：protected-green通行保护（继续前进）
                                7：yellow黄灯亮
                                8：flashing-yellow黄灯闪烁（警示灯） */
    uint16_t    nextDurationTime; /* 下个灯色的周期时间  红变绿剩余时间/绿变红剩余时间，0~36001 无效值36001，单位0.1s */

};

/* EV_GSENTRY_ADAS_OBJECT_VEHICLE_REPORT */
struct AdasObjVehInfo
{
    uint64_t    timeStamp;      /* 时间戳, 取值0~2^64-1 */
    uint8_t     localId;        /* 目标车辆标识ID，0~255 无效值0 */
    uint8_t     objectSource;   /* 目标数据来源，取值0~7
                                0:未知；
                                1:RSU；
                                2:OBU；
                                3:视频传感器；
                                4：微波雷达传感器；
                                5：地磁线圈传感器；
                                6：激光雷达传感器；
                                7：2类或以上感知数据的融合结果；
                                其他预留 */
    uint8_t     vehicleClass;   /* 目标车辆类型, 取值0~255
                                0：unknown 未知
                                1：special特殊用途
                                10：passengerUnknown 乘客车辆(默认)
                                11：passengerBasic 乘客车辆基本类型
                                20：lightTruckUnknown 轻型卡车（默认）
                                21：lightTruckBasicn 轻型卡车基本类型
                                25：truckUnknown 卡车（默认）
                                26：truckBasicn 卡车基本类型
                                27：truck-axleCnt2Basicn 两轴，六个轮胎单单元
                                28：truck-axleCnt3Basic 三轴，单个单元
                                29：truck-axleCnt4Basic 四轴或更多轴，单个单元
                                30：truck-axleCnt4TrailerBasic 四轴或更少轴，单拖车
                                31：truck-axleCnt5TrailerBasic 五轴或更少轴，单拖车
                                32：truck-axleCnt6TrailerBasic 六轴或更多轴，单拖车
                                33：truck-axleCnt5MultiTrailerBasic 五轴或以下，多拖车
                                34：truck-axleCnt6MultiTrailer Basic 六轴，多拖车
                                35：truck-axleCnt7MultiTrailer Basic 七轴或更多轴，多拖车
                                40：motor-TypeUnknownBasic 摩托车（默认类型）
                                41：motorcycle-TypeOtherBasic 摩托车（基本类型）
                                42：motorcycle-Cruiser-StandardBasic
                                43：motorcycle-SportUncladBasic
                                44：motorcycle-SportTouringBasic
                                45：motorcycle-SuperSportBasic
                                46：motorcycle-TouringBasic
                                47：motorcycle-TrikeBasic
                                48：motorcycle-wPassengersBasic*/
    VehicleSize size;
    uint16_t    objectHeadingAngle;     /* 目标车辆航向角为运动方向与正北方向的顺时针夹角, 取值0~28800，单位0.0125° */
    uint16_t    objectYawAngle;         /* 目标车辆横摆角速度顺时针旋转为正，逆时针为负, 取值-32767~32768，无效值32768，单位0.01°/s */
    uint8_t     gear;                   /* 车辆档位状态,取值0~7
                                        0 Neutral：空档
                                        1 Park：停止档
                                        2 ForwardGears：前进档
                                        3 ReverseGears：倒档 */
    int8_t      steeringWheelAngle;     /* 方向盘转角。向右为正，向左为负,-126~127,127为无效值,单位1.5° */
    uint8_t     remoteLight;            /* 转向灯信息，取值0~2
                                        0 off ：未开
                                        1 leftTurnSignalOn：左转灯亮
                                        2 rightTurnSignalOn：右转灯亮 */
    VehicleBrakes vehicleBrakes;
    uint8_t     stabilityControlStatus; /* 定义车辆动态稳定控制系统状态。
                                        0 :Unavailable 系统未装备或不可用。
                                        1 :Off 系统处于关闭状态。
                                        2 :On 系统处于开启状态，但未触发。
                                        3 :Engaged 系统被触发，处于作用状态。 */
    uint8_t     tractionControlStatus;  /* 牵引力控制系统实时状态。
                                        0 :Unavailable 系统未装备或不可用。
                                        1 :Off 系统处于关闭状态。
                                        2 :On 系统处于开启状态，但未触发。
                                        3 :Engaged 系统被触发，处于作用状态。*/
    uint8_t     espStatus;              /* 车身稳定性系统状态
                                        0 :Unavailable 系统未装备或不可用。
                                        1 :Off 系统处于关闭状态。
                                        2 :On 系统处于开启状态，但未触发。
                                        3 :Engaged 系统被触发，处于作用状态。 */
    uint8_t     ldwStatus;              /* 车道偏离预警
                                        0 :Unavailable 系统未装备或不可用。
                                        1 :Off 系统处于关闭状态。
                                        2 :On 系统处于开启状态，但未触发。
                                        3 :Engaged 系统被触发，处于作用状态。 */
    uint16_t            speed;          /* 车辆或其他交通参与者的速度大小。取值0~8191,无效值8191,单位0.02m/s */
    VehicleAccelSet     accelSet;
    Position            vehicelPos;
    ObjMapAddResult     mapInfo;
};

/* EV_GSENTRY_ADAS_EGOVEHI_MAPINFO_REPORT */
struct EgoVehMapInfo
{
    bool            MapInfoValid;
    CurLink         curLink;
    CurLane         curLane;
    DownLinksList   downLinksList[CONNECTION_NUM_PER_LINK];
    AdjacentLane    adjacentLane;
};

/* EV_GSENTRY_ADAS_OBJVEHI_MAPINFO_REPORT */
struct ObjVehMapInfo
{
    uint8_t     localId;            /* 道路ID（为快速匹配远车belongLinkID，该ID号自定义，不采用标准的上游和下游节点号）,0～255 */
    BelongsLink belongsLink;        /* 当前车辆所属道路信息 */
};

/* EV_GSENTRY_ADAS_WARNING_REPORT */
struct WarningInfo
{
    uint8_t     warningType;        /* WarningType：预警类型 取值0~3,0:无预警1:FCW;2:ICW;3:LTA; */
    uint8_t     level;              /* 预警等级，取值1~2,1:预备碰撞；2:碰撞  ；其他预留 */
    uint16_t    remoteLocalId;      /* 预警车辆本地ID（根据ID检索周围远车数据库里相应ID，获取详细车辆参数）,取值0~65535,数值0～255预留 */
    uint16_t    remoteBsmId;        /* 预警车辆BSM-ID,取值0~65535,数值0～255预留 */
    int16_t     objectCollisionTTC; /* 实时碰撞TTC,-100~100,无效值：-1,单位0.01s */
};

#endif /* F58F3DBF_4029_4C5F_885B_23B0F07B1B05 */
