import socket
from struct import pack, unpack
import pytest
import logging
import select
from collections import OrderedDict
import time
# from ctypes import *
from common import *
import ctypes



ipsoc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ipsoc.connect(('www.baidu.com', 80))
# target = ipsoc.getsockname()[0]
target = '0.0.0.0'
ipsoc.close()
com_port = 35558
v2x_port = 50600


v2x_msg_response_type_map = {
    MsgType.V2X_STATUS  : V2x_gSentryStatus,
    MsgType.V2X_SPAT    : Cdd_CurntLaneTrafficLightInfo,
    MsgType.V2X_WARN_INFO    : Cdd_gSentryWarningInfo,
    MsgType.V2X_REMOTE_VEHICLE    : Cdd_AllObjectVehicles
}


v2x_msg_response_len_map = {
    MsgType.V2X_STATUS  : ctypes.sizeof(V2x_gSentryStatus),
    MsgType.V2X_SPAT    : ctypes.sizeof(Cdd_CurntLaneTrafficLightInfo),
    MsgType.V2X_WARN_INFO    : ctypes.sizeof(Cdd_gSentryWarningInfo),
    MsgType.V2X_REMOTE_VEHICLE    : ctypes.sizeof(Cdd_AllObjectVehicles)
}


class MockV2xFusionServer:
    def __init__(self, server, port):
        self.v2x_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.v2x_socket.bind((target, port))

    def listen(self):
        self.v2x_socket.listen(1)
        conn,addr=self.v2x_socket.accept()   #接受TCP连接，并返回新的套接字与IP地址
        self.conection = conn

    def recv_once(self, length):
        ready = select.select([self.conection], [], [], 2.0)
        if ready[0]:
            recv_data = self.conection.recv(length)
            return recv_data

    def recv_one(self, msg_type):
        ready = select.select([self.conection], [], [], 2.0)
        if ready[0]:
            recv_data = self.conection.recv(v2x_msg_response_len_map[msg_type])
            return v2x_msg_response_type_map[msg_type].from_buffer_copy(recv_data)

        
    

    def send_once(self, msgId, payload):
        body = bytearray(payload)
        header = pack('=I2HI2HQ', 0x00FF0000, msgId, 0xffff, len(body), 0, 0, 0)
        self.conection.send(header + body)
    
    def close(self):
        self.conection.close()
        self.v2x_socket.close()



log = logging.getLogger(__name__)


config = {
    MsgType.V2X: ''
}

com_client = ComClient(target, com_port)
gSentry_server = MockV2xFusionServer(target, v2x_port)

                

@pytest.fixture(scope="session", autouse=True)
def entry():
    gSentry_server.listen()
    com_client.connect()

    yield

    gSentry_server.close()
    com_client.close()


class TestClassGsentryProxy:

    def test_gSentry_status(self):
        """模拟gSentry注入status数据,观察brown-sugar软件接收的数据是否和预期一致"""
        log.info(f"STEP01: send gSentry status : normal ")
        status = V2x_gSentryStatus()
        status.gSentryStatus = 2
        status.faultStatus = False
        time.sleep(0.05)
        gSentry_server.send_once(0x100, status)
        time.sleep(0.05)

        log.info(f"STEP02: check the response, gSentry status should be normal")
        recv_data = gSentry_server.recv_one(MsgType.V2X_STATUS)
        assert recv_data.gSentryStatus == status.gSentryStatus
        assert recv_data.faultStatus   == status.faultStatus

    def test_normal_spatinfo_with_current_green_light(self):
        """模拟gSentry注入交通灯数据,观察brown-sugar软件,是否只处理第一个红绿灯信息并正确计算红绿灯时间"""
        log.info(f"STEP1: send spat info : green ")

        all_spat_info = V2x_allSpatInfo()
        all_spat_info.spat_info[0].timeStamp = 0xffffffffffffffff
        all_spat_info.spat_info[0].spatInfoValid = True
        all_spat_info.spat_info[0].belongsNodeId = 1
        all_spat_info.spat_info[0].phaseID = 1
        all_spat_info.spat_info[0].lightState    = 5
        all_spat_info.spat_info[0].curCoutingTime = 70
        all_spat_info.spat_info[0].nextLight = 7
        all_spat_info.spat_info[0].nextDurationTime = 20
        time.sleep(0.05)
        gSentry_server.send_once(0x102, all_spat_info)
        time.sleep(0.05)

        log.info(f"STEP2: check the response, current light should be green")
        recv_data = gSentry_server.recv_one(MsgType.V2X_SPAT)
        # trafficLightSt, rsv1, rsv2, rsv3, redTime, greenTime, yellowTime = unpack('=4B3f', recv_data)
        assert recv_data.greenTime == 0
        assert recv_data.yellowTime   == all_spat_info.spat_info[0].curCoutingTime
        assert recv_data.redTime   == all_spat_info.spat_info[0].curCoutingTime + all_spat_info.spat_info[0].nextDurationTime

    def test_gSentry_warning_info(self):
        """模拟gSentry注入3个告警信息,观察brown-sugar软件,正确解出第一包告警的告警类型和车辆ID"""
        log.info(f"STEP1: send v2x warning info : fcw collision")
        warn = V2x_allWarnInfo()
        warn.warnInfo[0].warningType = 1
        warn.warnInfo[0].level = 2
        warn.warnInfo[0].remoteLocalId = 85
        warn.warnInfo[0].remoteBsmId = 14
        warn.warnInfo[0].objectCollisionTTC    = 53
        time.sleep(0.05)
        gSentry_server.send_once(0x302, warn)
        time.sleep(0.05)

        log.info(f"STEP2: check the response, current warning should be fcw collision")
        recv_data = gSentry_server.recv_one(MsgType.V2X_WARN_INFO)
        assert recv_data.warningType == warn.warnInfo[0].warningType
        assert recv_data.level   == warn.warnInfo[0].level
        assert recv_data.targetID   == warn.warnInfo[0].remoteLocalId

    def test_gSentry_object_vehicle_in_across_info(self):
        """模拟gSentry注入1个远车信息,观察brown-sugar软件,正确计算远车在本车车辆坐标系下的位移、速度和加速度--十字路口场景"""
        # 本用例测试数据从prescan获取，因此校验值使用魔法数
        # 本车long 114.0016624, lat 29.9985554，X 160.41， Y -160.13, 朝正北方向行驶
        # 远车long 114.0017942, lat 29.9987438，X 173.11， Y -139.25, 朝正西方向行驶, 前向速度10m/s，无转弯
        # 远车在本车坐标系描述为：                 X 20.88，   Y -12.7, 速度10m/s
        log.info(f"STEP1: send v2x host vehicle info")
        host = V2x_HostVehiInfo()
        host.latitude = 2099985554
        host.longitude = 2940016624
        host.elevation = 40000
        host.objectHeadingAngle = 0
        host.isHostPosValid = True
        com_client.send_once(host)
        time.sleep(0.5)

        log.info(f"STEP2: send v2x object vehicle info")
        obj_vehicles = V2x_AllAdasObjVehInfo()
        obj_vehicles.adasObjVehInfo[0].timeStamp = 14676655565
        obj_vehicles.adasObjVehInfo[0].localId   = 56
        obj_vehicles.adasObjVehInfo[0].objectSource = 2
        obj_vehicles.adasObjVehInfo[0].vehicleClass = 11
        obj_vehicles.adasObjVehInfo[0].size.length  = 300
        obj_vehicles.adasObjVehInfo[0].size.width   = 250
        obj_vehicles.adasObjVehInfo[0].size.width   = 150
        obj_vehicles.adasObjVehInfo[0].objectHeadingAngle = 21600
        obj_vehicles.adasObjVehInfo[0].objectYawAngle = 0
        obj_vehicles.adasObjVehInfo[0].gear = 2
        obj_vehicles.adasObjVehInfo[0].steeringWheelAngle = 0
        obj_vehicles.adasObjVehInfo[0].remoteLight = 0
        obj_vehicles.adasObjVehInfo[0].speed = 500
        obj_vehicles.adasObjVehInfo[0].accelSet.latitude = 0
        obj_vehicles.adasObjVehInfo[0].accelSet.longitude = 200
        obj_vehicles.adasObjVehInfo[0].vehicelPos.longitude = 1140017942
        obj_vehicles.adasObjVehInfo[0].vehicelPos.latitude  = 299987438
        obj_vehicles.adasObjVehInfo[0].vehicelPos.elevation = 40000
        gSentry_server.send_once(0x103, obj_vehicles)
        time.sleep(0.05)

        log.info(f"STEP3: check the response")
        # 取第一辆车数据进行比较
        all_object_vehicles = gSentry_server.recv_one(MsgType.V2X_REMOTE_VEHICLE)
        assert abs(all_object_vehicles.object_vehicles[0].De_dx_f32 - 20.88) < 0.2 
        assert abs(all_object_vehicles.object_vehicles[0].De_dy_f32 + 12.7) < 0.2 
        assert abs(all_object_vehicles.object_vehicles[0].De_vy_f32 - obj_vehicles.adasObjVehInfo[0].speed * 0.02) < 0.2 
        assert abs(all_object_vehicles.object_vehicles[0].De_ax_f32) < 0.2 
        assert abs(all_object_vehicles.object_vehicles[0].De_ay_f32 - obj_vehicles.adasObjVehInfo[0].accelSet.longitude * 0.01) < 0.2

    def test_gSentry_object_vehicle_in_ramp_info(self):
        """模拟gSentry注入1个远车信息,观察brown-sugar软件,正确计算远车在本车车辆坐标系下的位移、速度和加速度--匝道汇入"""
        # 本用例测试数据从prescan获取，因此校验值使用魔法数，本车沿正北直行，远程匝道行驶汇入，匝道与正北方向的逆时针夹角为20°
        # 本车long 114.0016099, lat 29.9964583，X 155.34， Y -392.60, 朝正北方向行驶
        # 远车long 114.0018903, lat 29.9961432，X 182.40， Y -427.53, 沿匝道行驶, 前向速度10m/s，无转弯
        # 远车在本车坐标系描述为：                 X -34.94，   Y -27.06, 速度vx:10 * cos(20°) = 9.3969  vy:10 * sin(20°) = 3.4202
        log.info(f"STEP1: send v2x host vehicle info")
        host = V2x_HostVehiInfo()
        host.latitude = 2099964584
        host.longitude = 2940016099
        host.elevation = 40000
        host.objectHeadingAngle = 0
        host.isHostPosValid = True
        com_client.send_once(host)
        time.sleep(0.5)

        log.info(f"STEP2: send v2x object vehicle info")
        obj_vehicles = V2x_AllAdasObjVehInfo()
        obj_vehicles.adasObjVehInfo[0].timeStamp = 14676655565
        obj_vehicles.adasObjVehInfo[0].localId   = 56
        obj_vehicles.adasObjVehInfo[0].objectSource = 2
        obj_vehicles.adasObjVehInfo[0].vehicleClass = 11
        obj_vehicles.adasObjVehInfo[0].size.length  = 300
        obj_vehicles.adasObjVehInfo[0].size.width   = 250
        obj_vehicles.adasObjVehInfo[0].size.width   = 150
        obj_vehicles.adasObjVehInfo[0].objectHeadingAngle = 27200
        obj_vehicles.adasObjVehInfo[0].objectYawAngle = 0
        obj_vehicles.adasObjVehInfo[0].gear = 2
        obj_vehicles.adasObjVehInfo[0].steeringWheelAngle = 0
        obj_vehicles.adasObjVehInfo[0].remoteLight = 0
        obj_vehicles.adasObjVehInfo[0].speed = 500
        obj_vehicles.adasObjVehInfo[0].accelSet.latitude = 0
        obj_vehicles.adasObjVehInfo[0].accelSet.longitude = 200
        obj_vehicles.adasObjVehInfo[0].vehicelPos.longitude = 1140018903
        obj_vehicles.adasObjVehInfo[0].vehicelPos.latitude  = 299961432
        obj_vehicles.adasObjVehInfo[0].vehicelPos.elevation = 40000
        gSentry_server.send_once(0x103, obj_vehicles)
        time.sleep(0.05)

        log.info(f"STEP3: check the response")
        # 取第一辆车数据进行比较
        all_object_vehicles = gSentry_server.recv_one(MsgType.V2X_REMOTE_VEHICLE)
        assert abs(all_object_vehicles.object_vehicles[0].De_dy_f32 + 27.06) < 0.2 
        assert abs(all_object_vehicles.object_vehicles[0].De_dx_f32 + 34.93) < 0.2 
        assert abs(all_object_vehicles.object_vehicles[0].De_vx_f32 - 9.3969) < 0.02 
        assert abs(all_object_vehicles.object_vehicles[0].De_vy_f32 - 3.4202) < 0.02 
        assert abs(all_object_vehicles.object_vehicles[0].De_ax_f32 - 1.8794) < 0.02 
        assert abs(all_object_vehicles.object_vehicles[0].De_ay_f32 - 0.684) < 0.02

    def test_gSentry_and_camera_coupled_objects_info(self):
        """模拟gSentry注入1个远车信息,camera注入50个障碍物信息,观察brown-sugar软件,正确处理v2x+camera数据并传出"""
        log.info(f"STEP1: send v2x host vehicle info")
        host = V2x_HostVehiInfo()
        host.latitude = 2099964584
        host.longitude = 2940016099
        host.elevation = 40000
        host.objectHeadingAngle = 0
        host.isHostPosValid = True
        com_client.send_once(host)
        time.sleep(0.5)

        log.info(f"STEP2: send v2x object vehicle info")
        obj_vehicles = V2x_AllAdasObjVehInfo()
        obj_vehicles.adasObjVehInfo[0].timeStamp = 14676655565
        obj_vehicles.adasObjVehInfo[0].localId   = 56
        obj_vehicles.adasObjVehInfo[0].objectSource = 2
        obj_vehicles.adasObjVehInfo[0].vehicleClass = 11
        obj_vehicles.adasObjVehInfo[0].size.length  = 300
        obj_vehicles.adasObjVehInfo[0].size.width   = 250
        obj_vehicles.adasObjVehInfo[0].size.width   = 150
        obj_vehicles.adasObjVehInfo[0].objectHeadingAngle = 27200
        obj_vehicles.adasObjVehInfo[0].objectYawAngle = 0
        obj_vehicles.adasObjVehInfo[0].gear = 2
        obj_vehicles.adasObjVehInfo[0].steeringWheelAngle = 0
        obj_vehicles.adasObjVehInfo[0].remoteLight = 0
        obj_vehicles.adasObjVehInfo[0].speed = 500
        obj_vehicles.adasObjVehInfo[0].accelSet.latitude = 0
        obj_vehicles.adasObjVehInfo[0].accelSet.longitude = 200
        obj_vehicles.adasObjVehInfo[0].vehicelPos.longitude = 1140018903
        obj_vehicles.adasObjVehInfo[0].vehicelPos.latitude  = 299961432
        obj_vehicles.adasObjVehInfo[0].vehicelPos.elevation = 40000
        gSentry_server.send_once(0x103, obj_vehicles)
        time.sleep(0.5)
        all_object_vehicles = gSentry_server.recv_one(MsgType.V2X_REMOTE_VEHICLE)

        log.info(f"STEP3: send camera vehicle obstacles info")
        camera_data = Camera_Obstacles()
        camera_data.obstacle_num = 50
        for index in range(0, camera_data.obstacle_num):
            camera_data.obstacles[index].id = index + 1
            camera_data.obstacles[index].world_info.position.x = camera_data.obstacle_num - index
            camera_data.obstacles[index].world_info.position.y = camera_data.obstacle_num - index

        com_client.send_once(camera_data)
        time.sleep(0.5)


        log.info(f"STEP4: check the response")
        # 取第一辆车数据进行比较
        all_object_vehicles = com_client.recv_once(MsgType.IPC_OBJ_INFO)
        assert abs(all_object_vehicles.object_vehicles[0].De_dy_f32 + 27.06) < 0.2 
        assert abs(all_object_vehicles.object_vehicles[0].De_dx_f32 + 34.93) < 0.2 
        assert abs(all_object_vehicles.object_vehicles[0].De_vx_f32 - 9.3969) < 0.02 
        assert abs(all_object_vehicles.object_vehicles[0].De_vy_f32 - 3.4202) < 0.02 
        assert abs(all_object_vehicles.object_vehicles[0].De_ax_f32 - 1.8794) < 0.02 
        assert abs(all_object_vehicles.object_vehicles[0].De_ay_f32 - 0.684) < 0.02

        for number in range(21, 40):
            pre_id = all_object_vehicles.object_vehicles[number - 1].De_ID_u8
            pre_dx = all_object_vehicles.object_vehicles[number - 1].De_dx_f32
            pre_dy = all_object_vehicles.object_vehicles[number - 1].De_dy_f32
            next_id = all_object_vehicles.object_vehicles[number].De_ID_u8
            next_dx = all_object_vehicles.object_vehicles[number].De_dx_f32
            next_dy = all_object_vehicles.object_vehicles[number].De_dy_f32
            assert pre_id - 1 == next_id
            assert pre_dx * pre_dx + pre_dy * pre_dy < next_dx * next_dx + next_dy * next_dy
    



