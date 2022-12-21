import ctypes
from re import M
from struct import pack
import socket
from struct import pack, unpack
import select
from datetime import datetime
import collections
import logging
from ctypes import *
import time

log = logging.getLogger(__name__)

class MsgType:
    V2X = 0
    CAMERA = 1
    CAN = 2
    CDS = 3
    HMI_DUMMY = 4
    HMI_PAD = 5
    HMI_GSENTRY = 6
    IPC_DUMMY = 7
    IPC_GNSS_ACC = 8
    IPC_GNSS_GYRO = 9
    IPC_GNSS_HEADING_PITCH_ROLL = 10
    IPC_GNSS_HEIGHT_TIME = 11
    IPC_GNSS_LAT_LONG = 12
    IPC_GNSS_SPEED = 13
    IPC_GNSS_DATA_INFO = 14
    IPC_GNSS_STD = 15
    IPC_GNSS_UTC = 16
    IPC_CAN = 17
    IPC_EVH = 18
    IPC_HMI_INFO = 19
    IPC_SYS_ERROR = 20
    IPC_HMI_CTRL = 21
    IPC_CAM_OBJ = 22
    IPC_V2X_OBJ = 23
    IPC_LANE_INFO = 24
    IPC_GSENTRY_WARN = 25
    IPC_DIS2ENDLANE = 26
    IPC_TRAFFIC_LIGHT_INFO = 27
    XDS_DUMMY = 28
    V2X_STATUS = 29
    V2X_SPAT = 30
    V2X_WARN_INFO = 31
    V2X_REMOTE_VEHICLE = 32
    CDD_CAMERA = 33
    
class HmiCtrl(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        #  ('De_AEB_SwtRequest_u8', c_ubyte),
        #  ('De_FCW_SnvtySet_u8', c_ubyte),
        #  ('De_TimeGapSet_f32', c_float),
        #  ('De_ResumeSwitch_u8', c_ubyte),
        #  ('De_CACCSWOn_u8', c_ubyte),
        #  ('De_IDA_ResSw_u8"', c_ubyte),
        #  ('De_IDA_CancelSw_u8', c_ubyte),
        #  ('De_VehSpdSetSw_u8', c_ubyte),
        #  ('De_CACCcancelSW_u8', c_ubyte),
        #  ('De_Vset_f32', c_float),
        #  ('De_VehSpd_f32', c_float),
        #  ('De_RequestCounter_u32', c_uint32),
         ('De_AEB_SwtRequest_u8', c_ubyte),
         ('De_FCW_SnvtySet_u8', c_ubyte),
         ('De_CACC_Switch_u8', c_ubyte),
         ('De_IDA_Switch_u8', c_ubyte),
         ('De_CACC_Resume_u8', c_ubyte),
         ('De_CACC_Cancel_u8"', c_ubyte),
         ('De_TimeGapSet_u8', c_ubyte),
         ('De_VSet_f32', c_float),
         ('De_VDis_f32', c_float),
         ('De_RequestCounter_u32', c_uint32),
    ]

class HmiInfo(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ('De_ADAS_FCW_u8', c_ubyte),
        ('De_gSentry_Warning_u8', c_ubyte),
        ('De_FCW_AEB_FuncSts_u8', c_ubyte),
        ('De_AEB_Triger_u8', c_ubyte),
        ('De_AEB_Response_u8', c_ubyte),
        ('De_CACC_CACCStatus_u8', c_ubyte),
        ('De_CACC_ACCStatus_u8', c_ubyte),
        ('De_CACC_IDAStatus_u8', c_ubyte),
        ('De_CACC_Decision_u8', c_ubyte),
        ('De_CACC_CACCOn_Rsp_u8', c_ubyte),
        ('De_CACC_CACCOff_Rsp_u8', c_ubyte),
        ('De_CACC_CACCResume_Rsp_u8', c_ubyte),
        ('De_CACC_SpeedSet_Rsp_u8', c_ubyte),
        ('De_CACC_TimeGapSet_Rsp_u8', c_ubyte),
        ('De_CACC_IDAOn_Rsp_u8', c_ubyte),
        ('De_CACC_IDAOff_Rsp_u8', c_ubyte),
        ('De_CACC_SportMode_Rsp_u8', c_ubyte),
        ('De_ResponseCounter_u32', c_uint32),
    ]
    
class GNSS_HeadingPitchRoll(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ("pitch", c_float), 
        ("roll", c_float), 
        ("heading", c_float)
    ]
    
class GNSS_LatitudeLongitude(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ("lat", c_float), 
        ("long", c_float)
    ]

class EvhInfo(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ('De_ego_vxMs_f32', c_float),
        ('De_ego_axMs2_f32', c_float),
        ('De_ego_ayMs2_f32', c_float),
        ('De_Kaptraj_f32', c_float),
        ('De_YawrateFilter_f32', c_float),
        ('De_RoadSlope_f32', c_float),
    ]
#  ************************ v2x struct ************************
class V2x_HostVehiInfo(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("latitude", c_uint32), 
        ("longitude", c_uint32), 
        ("elevation", c_uint32), 
        ("objectHeadingAngle", c_uint16), 
        ("isHostPosValid", c_bool)
    ]
class V2x_gSentryStatus(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("gSentryStatus", c_uint8), 
        ("faultStatus", c_bool)
    ]
class V2x_oneSpatInfo(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("timeStamp", c_uint64), 
        ("spatInfoValid", c_bool), 
        ("belongsNodeId", c_int32), 
        ("phaseID", c_uint8), 
        ("lightState", c_uint8),
        ("curCoutingTime", c_uint16), 
        ("nextLight", c_uint8), 
        ("nextDurationTime", c_uint16)
    ]

class V2x_allSpatInfo(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("spat_info", V2x_oneSpatInfo * 5)
    ]

class V2x_warnInfo(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("warningType", c_uint8), 
        ("level", c_uint8),
        ("remoteLocalId", c_uint16), 
        ("remoteBsmId", c_uint16), 
        ("objectCollisionTTC", c_int16)
    ]

class V2x_allWarnInfo(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("warnInfo", V2x_warnInfo * 3)
    ]

class V2x_VehicleSize(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("length", c_uint16), 
        ("width", c_uint16), 
        ("height", c_uint8)
    ]

class V2x_VehicleBrakes(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("brakePedalStatus", c_uint8), 
        ("absBrakes", c_uint8), 
        ("brakeAppliedStatus", c_uint8), 
        ("brakeBoostApplied", c_uint8), 
        ("auxiliaryBrakeStatus", c_uint8)
    ]

class V2x_VehicleAccelSet(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("longitude", c_int16), 
        ("latitude", c_int16)
    ]

class CdsData(LittleEndianStructure):
    _pack_ = 4
    _fields_ =  [
        ('transmissionState', c_uint32),
        ('steeringWheelAngle', c_uint32),
        ('brakePedalStatus', c_uint32),
        ('brakeAppliedStatus', c_uint32),
        ('tractionControlStatus', c_uint32),
        ('antiLockBrakeStatus', c_uint32),
        ('stabilityControlStatus', c_uint32),
        ('brakeBoostApplied', c_uint32),
        ('auxiliaryBrakesStatus', c_uint32),
        ('eventHazardLights', c_uint32),
        ('eventABSactivated', c_uint32),
        ('eventTractionControlLoss', c_uint32),
        ('eventStabilityControlactivated', c_uint32),
        ('eventHardBraking', c_uint32),
        ('eventFlatTire', c_uint32),
        ('eventDisabledVehicle', c_uint32),
        ('eventAirBagDeployment', c_uint32),
        ('eventWipersStatus', c_uint32),
        ('exteriorLights', c_uint32),
        ('vehicleSpeed', c_uint32),
        ('enduranceMileage', c_uint32), 
    ]

# class HmiCtrl(LittleEndianStructure):
#     _pack_ = 4
#     _fields_ = [
#          ('De_AEB_SwtRequest_u8', c_ubyte),
#          ('De_FCW_SnvtySet_u8', c_ubyte),
#          ('De_TimeGapSet_f32', c_float),
#          ('De_ResumeSwitch_u8', c_ubyte),
#          ('De_CACCSWOn_u8', c_ubyte),
#          ('De_IDA_ResSw_u8"', c_ubyte),
#          ('De_IDA_CancelSw_u8', c_ubyte),
#          ('De_VehSpdSetSw_u8', c_ubyte),
#          ('De_CACCcancelSW_u8', c_ubyte),
#          ('De_Vset_f32', c_float),
#          ('De_VehSpd_f32', c_float),
#          ('De_RequestCounter_u32', c_uint32),
#     ]

# class HmiInfo(LittleEndianStructure):
#     _pack_ = 4
#     _fields_ = [
#         ('De_ADAS_FCW_u8', c_ubyte),
#         ('De_gSentry_Warning_u8', c_ubyte),
#         ('De_FCW_AEB_FuncSts_u8', c_ubyte),
#         ('De_AEB_Triger_u8', c_ubyte),
#         ('De_AEB_Response_u8', c_ubyte),
#         ('De_CACC_CACCStatus_u8', c_ubyte),
#         ('De_CACC_ACCStatus_u8', c_ubyte),
#         ('De_CACC_IDAStatus_u8', c_ubyte),
#         ('De_CACC_Decision_u8', c_ubyte),
#         ('De_CACC_CACCOn_Rsp_u8', c_ubyte),
#         ('De_CACC_CACCOff_Rsp_u8', c_ubyte),
#         ('De_CACC_CACCResume_Rsp_u8', c_ubyte),
#         ('De_CACC_SpeedSet_Rsp_u8', c_ubyte),
#         ('De_CACC_TimeGapSet_Rsp_u8', c_ubyte),
#         ('De_CACC_IDAOn_Rsp_u8', c_ubyte),
#         ('De_CACC_IDAOff_Rsp_u8', c_ubyte),
#         ('De_CACC_SportMode_Rsp_u8', c_ubyte),
#         ('De_ResponseCounter_u32', c_uint32),
#     ]
    
class GNSS_Acc(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ('acc_x', c_float),
        ('acc_y', c_float),
        ('acc_z', c_float),
    ]

class GNSS_Gyro(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ('gyro_x', c_float),
        ('gyro_y', c_float),
        ('gyro_z', c_float),
    ]

class GNSS_HeadingPitchRoll(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ("pitch", c_float), 
        ("roll", c_float), 
        ("heading", c_float)
    ]
    
class GNSS_HeightAndTime(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ('locatHeight', c_float),
        ('time', c_uint32),
    ]
    
class GNSS_LatitudeLongitude(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ("lat", c_float), 
        ("long", c_float)
    ]
    
class GNSS_Speed(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ('northSpd', c_float),
        ('eastSpd', c_float),
        ('toGroundSpd', c_float),
    ]
    

class GNSS_DataInfo(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ('gpsFlagPos', c_uint8),
        ('numSv', c_uint8),
        ('gpsFlagHeading', c_uint8),
        ('gpsAge', c_uint8),
        ('gpsStatus', c_uint8),
        ('status', c_uint8),
        ('reserved', c_uint32),
    ]

class GNSS_Std(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ('stdLat', c_float),
        ('stdLon', c_float),
        ('stdLocateHeight', c_float),
        ('stdHeading', c_float),
    ]

class GNSS_UTC(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ('utcYear', c_uint8),
        ('utcMonth', c_uint8),
        ('utcDay', c_uint8),
        ('utcHour', c_uint8),
        ('utcMin', c_uint8),
        ('utcSec', c_uint8),
        ('utcMilliSec', c_uint16),
        ('reserved', c_uint32),
    ]


class EvhInfo(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ('De_ego_vxMs_f32', c_float),
        ('De_ego_axMs2_f32', c_float),
        ('De_ego_ayMs2_f32', c_float),
        ('De_Kaptraj_f32', c_float),
        ('De_YawrateFilter_f32', c_float),
        ('De_RoadSlope_f32', c_float),
    ]


#  ************************ v2x struct ************************

class V2x_Position(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("latitude", c_int32), 
        ("longitude", c_int32), 
        ("elevation", c_int32)
    ]

class V2x_ObjMapAddResult(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("offsetTolink", c_int16), 
        ("offsetTolane", c_int16), 
        ("distToNode", c_uint32), 
        ("isAtAcross", c_bool), 
        ("linkRealtion", c_int8)
    ]

class V2x_OneAdasObjVehInfo(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("timeStamp", c_uint64), 
        ("localId", c_uint8),
        ("objectSource", c_uint8),
        ("vehicleClass", c_uint8),
        ("size", V2x_VehicleSize),
        ("objectHeadingAngle", c_uint16), 
        ("objectYawAngle", c_uint16), 
        ("gear", c_uint8),
        ("steeringWheelAngle", c_int8),
        ("remoteLight", c_uint8), 
        ("vehicleBrakes", V2x_VehicleBrakes), 
        ("stabilityControlStatus", c_uint8),
        ("tractionControlStatus", c_uint8),
        ("espStatus", c_uint8),
        ("ldwStatus", c_uint8),
        ("speed", c_uint16), 
        ("accelSet", V2x_VehicleAccelSet),
        ("vehicelPos", V2x_Position),
        ("targetClassification", c_uint8),
        ("mapInfo", V2x_ObjMapAddResult)
    ]

class V2x_AllAdasObjVehInfo(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("adasObjVehInfo", V2x_OneAdasObjVehInfo * 20)
    ]


#  ************************ camrea struct ************************
class Camera_Point(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("x", c_float),
        ("y", c_float)
    ]
class Camera_Line(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("valid", c_bool),
        ("id", c_uint32),
        ("life_time", c_uint32),
        ("type", c_uint32),
        ("conf", c_float),
        ("width", c_float),
        ("start_pt", Camera_Point),
        ("y_coeff", c_float * 4),
        ("t_max", c_float),
        ("color", c_uint32),
        ("marking", c_uint32),
        ("parsing_conf", c_float),
        ("rmse", c_float)
    ]
class Camera_Lines(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("timestamp", c_uint32),
        ("left", Camera_Line),
        ("left_left", Camera_Line),
        ("right", Camera_Line),
        ("right_right", Camera_Line),
        ("dtlc", c_float),
        ("ttlc", c_float)
    ]
class Camera_Velocity(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("vx", c_float),
        ("vy", c_float)
    ]
class Camera_Accleration(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("ax", c_float),
        ("ay", c_float)
    ]
class Camera_ObjCornerPoint(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("objCornerPoint_x", c_float),
        ("objCornerPoint_y", c_float),
        ("objDistInLane", c_float),
        ("objCutInFlag", c_uint32),
        ("objCutInLane", c_uint32),
        ("ll_type", c_uint32),
        ("rr_type", c_uint32),
        ("distance_to_ll", c_float),
        ("distance_to_rr", c_float)
    ]
class Camera_WorldSpaceInfo(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("yaw", c_float),
        ("vel", Camera_Velocity),
        ("length", c_float),
        ("width", c_float),
        ("height", c_float),
        ("position", Camera_Point),
        ("ttc", c_float),
        ("curr_lane", c_uint32),
        ("ettc", c_float),
        ("acc", c_float),
        ("motion_state", c_uint32),
        ("vel_abs_world", Camera_Velocity),
        ("acc_abs_world", Camera_Accleration),
        ("motion_category", c_uint32),
        ("position_type", c_uint32),
        ("yaw_rate", c_float),
        ("sigma_yaw", c_float),
        ("sigma_vel", c_float * 9),
        ("sigma_width", c_float),
        ("sigma_height", c_float),
        ("sigma_position", c_float * 9),
        ("sigma_length", c_float),
        ("conf_yaw", c_float),
        ("cipv", c_uint32),
        ("measurement_status", c_uint32),
        ("mid_angle", c_float),
        ("obj_corner_point", Camera_ObjCornerPoint),
        ("acc_ref", Camera_Accleration),
        ("mcp", c_uint32)
    ]
class Camera_Obstacle(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("id", c_uint32),
        ("timestamp", c_uint32),
        ("type", c_uint32),
        ("conf", c_uint32),
        ("life_time", c_uint32),
        ("world_info", Camera_WorldSpaceInfo),
        ("serial_number", c_uint32),
        ("select_level", c_uint32)
    ]
class Camera_Obstacles(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("obstacle_num", c_uint32),
        ("obstacles", Camera_Obstacle * 50),
        ("cipv_id", c_uint32),
        ("mcp_id", c_uint32)
    ]

class Cdd_CurntLaneTrafficLightInfo(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ("trafficLightSt", c_bool),
        ("redTime", c_float),
        ("greenTime", c_float),
        ("yellowTime", c_float)
    ]


class Cdd_gSentryWarningInfo(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ("warningType", c_uint8),
        ("level", c_uint8),
        ("targetID", c_uint8)
    ]


class Cdd_OneObjectVehicle(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ("De_Timestamp_u32", c_uint32),
        ("De_ID_u8", c_uint8),
        ("De_conf_f32", c_float),
        ("De_measurement_status_u8", c_uint8),
        ("De_life_time_u32", c_uint32),
        ("De_length_f32", c_float),
        ("De_width_f32", c_float),
        ("De_height_f32", c_float),
        ("De_Yaw_f32", c_float),
        ("De_conf_yaw_f32", c_float),
        ("De_yaw_rate_f32", c_float),
        ("De_dx_f32", c_float),
        ("De_dy_f32", c_float),
        ("De_vx_f32", c_float),
        ("De_vy_f32", c_float),
        ("De_ax_f32", c_float),
        ("De_ay_f32", c_float),
        ("De_dxVariance_f32", c_float),
        ("De_dyVariance_f32", c_float),
        ("De_vxVariance_f32", c_float),
        ("De_vyVariance_f32", c_float),
        ("De_axVariance_f32", c_float),
        ("De_ayVariance_f32", c_float),
        ("De_curr_lane_f32", c_float),
        ("De_ttc_f32", c_float),
        ("De_ettc_f32", c_float),
        ("De_CIPV_u8", c_uint8),
        ("De_source_u8", c_uint32),
        ("De_objectType_u8", c_uint8),
        ("De_objectMovingStatus_u8", c_uint8),
    ]


class Cdd_AllObjectVehicles(LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ("object_vehicles", Cdd_OneObjectVehicle * 40)
    ]

com_msg_map = {
    MsgType.IPC_HMI_CTRL : HmiCtrl,
    MsgType.IPC_HMI_INFO : HmiInfo,
    MsgType.IPC_GNSS_ACC : GNSS_Acc,
    MsgType.IPC_GNSS_GYRO : GNSS_Gyro,
    MsgType.IPC_GNSS_HEADING_PITCH_ROLL : GNSS_HeadingPitchRoll,
    MsgType.IPC_GNSS_HEIGHT_TIME : GNSS_HeightAndTime,
    MsgType.IPC_GNSS_LAT_LONG : GNSS_LatitudeLongitude,
    MsgType.IPC_GNSS_SPEED : GNSS_Speed,
    MsgType.IPC_GNSS_DATA_INFO : GNSS_DataInfo,
    MsgType.IPC_GNSS_STD : GNSS_Std,
    MsgType.IPC_GNSS_UTC : GNSS_UTC,
    MsgType.IPC_EVH : EvhInfo,
    MsgType.IPC_CAN: CdsData,
    MsgType.CAN : V2x_HostVehiInfo,
    MsgType.CAMERA : Camera_Obstacles,
    MsgType.IPC_OBJ_INFO: Cdd_AllObjectVehicles
}


com_tcp_header_pack_fmt = '>IIIQ'


class ComClient:
    def __init__(self, server, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server = server
        self.port = port

    def connect(self):
        self.socket.connect((self.server, self.port))
        self.socket.setblocking(False)
    
    def send_once(self, data, msg_type = None):
        body = bytearray(data)
        if msg_type is None:
            msg_type = next((m for m, t in com_msg_map.items() if t == type(data)))
        header = pack(com_tcp_header_pack_fmt, 0x00FF0000, len(body), msg_type, int(time.time()*10e8))
        #logging.info(len(body))
        #logging.info(' '.join([hex(x) for x in header + body]))
        self.socket.send(header + body)
    
    # def recv_once(self, expected_msg_type, timeout=3):
    #     start_time = datetime.now()
    #     while (datetime.now() - start_time).total_seconds() < timeout:
    #         ready = select.select([self.socket], [], [], 1.0)
    #         if ready[0]:
    #             self.socket.setblocking(True)
    #             recv_data = self.socket.recv(12)
    #             start_flag, length, msg_type = unpack(com_tcp_header_pack_fmt, recv_data)
    #             # print(f'{start_flag}, {length}, {msg_type}')
    #             if start_flag == 0x00FF0000 and length < 65536 :
    #                 if msg_type == expected_msg_type:
    #                     recv_data = self.socket.recv(length)
    #                     logging.info(' '.join([hex(x) for x in recv_data]))
    #                     return com_msg_map[msg_type].from_buffer_copy(recv_data)
    #                 else:
    #                     self.socket.recv(length)
    #             self.socket.setblocking(False)

    def recv_once(self, expected_msg_type, timeout=3):
        start_time = datetime.now()
        while (datetime.now() - start_time).total_seconds() < timeout:
            ready = select.select([self.socket], [], [], 1.0)
            if ready[0]:
                self.socket.setblocking(True)
                recv_data = self.socket.recv(12)
                start_flag, length, msg_type, timestamp = unpack(com_tcp_header_pack_fmt, recv_data)
                # print(f'{start_flag}, {length}, {msg_type}')
                if start_flag == 0x00FF0000 and length < 65536 :
                    if msg_type == expected_msg_type:
                        recv_data = self.socket.recv(length)
                        logging.info(' '.join([hex(x) for x in recv_data]))
                        return com_msg_map[msg_type].from_buffer_copy(recv_data)
                    else:
                        self.socket.recv(length)
                self.socket.setblocking(False)

    def recv_one_packet(self, length):
        ready = select.select([self.socket], [], [], 1.0)
        if ready[0]:
            self.socket.setblocking(True)
            recv_data = self.socket.recv(length)
            return recv_data
    
    def close(self):
        self.socket.close()