import socket
from struct import pack, unpack
import pytest
import logging
import select
from collections import OrderedDict
import time
from ctypes import *
from common import *


ipsoc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ipsoc.connect(('www.baidu.com', 80))
target = ipsoc.getsockname()[0]
ipsoc.close()
com_port = 35558

log = logging.getLogger(__name__)

com_client = ComClient(target, com_port)

                

@pytest.fixture(scope="session", autouse=True)
def entry():
    com_client.connect()

    yield

    com_client.close()


class TestClassCameraProxy:

    def test_gSentry_object_vehicle_info(self):
        """模拟4辆车辆信息,观察brown-sugar软件是否根据障碍物(车辆)与本车的距离进行排序"""
        log.info(f"STEP1: send 4 camera vehicle obstacles info")
        camera_data = Camera_Obstacles()
        camera_data.obstacle_num = 4
        for index in range(0, camera_data.obstacle_num):
            camera_data.obstacles[index].id = index + 1
            camera_data.obstacles[index].world_info.position.x = camera_data.obstacle_num - index
            camera_data.obstacles[index].world_info.position.y = camera_data.obstacle_num - index

        com_client.send_once(camera_data)
        time.sleep(0.5)

        log.info(f"STEP2: recieve arranged camera obstacles info ")

        all_object_vehicles = com_client.recv_once(MsgType.IPC_OBJ_INFO)

        # 循环校验有效障碍物
        for number in range(20, 20 + camera_data.obstacle_num):
            id = all_object_vehicles.object_vehicles[number].De_ID_u8
            dx = all_object_vehicles.object_vehicles[number].De_dx_f32
            dy = all_object_vehicles.object_vehicles[number].De_dy_f32
            assert id == camera_data.obstacles[camera_data.obstacle_num + 20 - number - 1].id
            assert dx == camera_data.obstacles[camera_data.obstacle_num + 20 - number - 1].world_info.position.x
            assert dy == camera_data.obstacles[camera_data.obstacle_num + 20 - number - 1].world_info.position.y


    def test_gSentry_object_vehicle_and_pedestrian_info(self):
        """模拟4辆车辆和1个行人信息,观察brown-sugar软件是否根据障碍物(车辆+行人)与本车的距离进行排序"""
        log.info(f"STEP1: send camera vehicle and pedestrian obstacles info")
        camera_vehicle_data = Camera_Obstacles()
        camera_vehicle_data.obstacle_num = 4
        for index in range(0, camera_vehicle_data.obstacle_num):
            camera_vehicle_data.obstacles[index].id = index + 1
            camera_vehicle_data.obstacles[index].world_info.position.x = camera_vehicle_data.obstacle_num - index
            camera_vehicle_data.obstacles[index].world_info.position.y = camera_vehicle_data.obstacle_num - index

        camera_pedestrian_data = Camera_Obstacles()
        camera_pedestrian_data.obstacle_num = 1
        camera_pedestrian_data.obstacles[0].id = 7
        camera_pedestrian_data.obstacles[0].world_info.position.x = 1
        camera_pedestrian_data.obstacles[0].world_info.position.y = -1.5

        com_client.send_once(camera_vehicle_data)
        com_client.send_once(camera_pedestrian_data)
        time.sleep(0.5)

        log.info(f"STEP2: recieve arranged camera obstacles info ")
        # 把消息头部数据丢掉
        all_object_vehicles = com_client.recv_once(MsgType.IPC_OBJ_INFO)
        assert all_object_vehicles.object_vehicles[20].De_ID_u8 == camera_vehicle_data.obstacles[3].id
        assert all_object_vehicles.object_vehicles[20].De_dx_f32  == camera_vehicle_data.obstacles[3].world_info.position.x
        assert all_object_vehicles.object_vehicles[20].De_dy_f32  == camera_vehicle_data.obstacles[3].world_info.position.y

        assert all_object_vehicles.object_vehicles[21].De_ID_u8 == camera_pedestrian_data.obstacles[0].id
        assert all_object_vehicles.object_vehicles[21].De_dx_f32 == camera_pedestrian_data.obstacles[0].world_info.position.x
        assert all_object_vehicles.object_vehicles[21].De_dy_f32 == camera_pedestrian_data.obstacles[0].world_info.position.y

        # 循环校验有效障碍物
        for number in range(22,20 + camera_vehicle_data.obstacle_num + camera_pedestrian_data.obstacle_num):
            id = all_object_vehicles.object_vehicles[number].De_ID_u8
            dx = all_object_vehicles.object_vehicles[number].De_dx_f32
            dy = all_object_vehicles.object_vehicles[number].De_dy_f32
            assert id == camera_vehicle_data.obstacles[camera_vehicle_data.obstacle_num + 20 - number].id
            assert dx == camera_vehicle_data.obstacles[camera_vehicle_data.obstacle_num + 20 - number].world_info.position.x
            assert dy == camera_vehicle_data.obstacles[camera_vehicle_data.obstacle_num + 20 - number].world_info.position.y

        log.info(f"STEP3: multi-topic camera obstacles is arranged from small to large! ")


    def test_gSentry_max_object_vehicle_info(self):
        """模拟50辆车辆信息,观察brown-sugar软件是否只保存距离本车最近的20个障碍物信息，且从小到大排列"""
        log.info(f"STEP1: send camera vehicle obstacles info")
        camera_data = Camera_Obstacles()
        camera_data.obstacle_num = 50
        for index in range(0, camera_data.obstacle_num):
            camera_data.obstacles[index].id = index + 1
            camera_data.obstacles[index].world_info.position.x = camera_data.obstacle_num - index
            camera_data.obstacles[index].world_info.position.y = camera_data.obstacle_num - index

        com_client.send_once(camera_data)
        time.sleep(0.5)

        log.info(f"STEP2: recieve arranged camera obstacles info ")
        all_object_vehicles = com_client.recv_once(MsgType.IPC_OBJ_INFO)
        for number in range(21, 40):
            pre_id = all_object_vehicles.object_vehicles[number - 1].De_ID_u8
            pre_dx = all_object_vehicles.object_vehicles[number - 1].De_dx_f32
            pre_dy = all_object_vehicles.object_vehicles[number - 1].De_dy_f32
            next_id = all_object_vehicles.object_vehicles[number].De_ID_u8
            next_dx = all_object_vehicles.object_vehicles[number].De_dx_f32
            next_dy = all_object_vehicles.object_vehicles[number].De_dy_f32
            assert pre_id - 1 == next_id
            assert pre_dx * pre_dx + pre_dy * pre_dy < next_dx * next_dx + next_dy * next_dy

        log.info(f"STEP3: 20(max number) camera obstacles is arranged from small to large! ")
    



