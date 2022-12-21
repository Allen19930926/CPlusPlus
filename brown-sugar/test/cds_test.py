from asyncio import sleep
from itertools import count
import socket
from struct import pack, unpack
import pytest

from common import *
import logging
import select
from datetime import datetime
from collections import OrderedDict
import time
import json
import base64




class MockCdsClient:
    def __init__(self, server, port):
        self.cds_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server = server
        self.port = port

    def connect(self):
        self.cds_socket.connect((self.server, self.port))
        self.cds_socket.setblocking(False)

    def recv_once(self, timeout=3):
        start_time = datetime.now()
        while (datetime.now() - start_time).total_seconds() < timeout:
            ready = select.select([self.cds_socket], [], [], 2.0)
            if ready[0]:
                recv_data = self.cds_socket.recv(28)
                start_flag, rsv1, rsv2, timestamp, msgid, matchId, length = unpack('<HhIQIII', recv_data) 
                if start_flag == 65280 and length < 65536:
                    recv_data = self.cds_socket.recv(length)
                    #logging.info('  '.join([hex(x) for x in recv_data]))
                    package = json.loads(recv_data)
                    #logging.info(package)
                    if package["Data"]:
                        return package
                #break
                    
    def close(self):
        self.cds_socket.close()

    def clear(self):
        while True:
            inputready, o, e = select.select([self.cds_socket], [], [], 0)
            if len(inputready)==0: 
                break
            for s in inputready: 
                s.recv(1)



log = logging.getLogger(__name__)

target = "127.0.0.1"
com_port = 35558
cds_port = 35551

com_client = ComClient(target, com_port)
cds_client = MockCdsClient(target, cds_port)
        

@pytest.fixture(scope="session", autouse=True)
def entry():
    com_client.connect()
    cds_client.connect()

    yield

    com_client.close()
    cds_client.close()

'''src_Candata：CAN数据初始值'''
src_can_data = [1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

class TestClasscdsProxy:
    def test_cds_send_data(self):
        """CDS_TEST_001 IPC注入can数据，观察CDS客户端接收的数据是否和预期一致"""
        '''
        "transmissionState", "steeringWheelAngle", "brakePedalStatus", "brakeAppliedStatus", "tractionControlStatus", "antiLockBrakeStatus",
        "stabilityControlStatus", "brakeBoostApplied", "auxiliaryBrakesStatus", "eventHazardLights", "eventABSactivated", "eventTractionControlLoss", 
        "eventStabilityControlactivated", "eventHardBraking", "eventFlatTire", "eventDisabledVehicle", "eventAirBagDeployment", "eventWipersStatus",
        "exteriorLights", "vehicleSpeed", "enduranceMileage"
        '''
        
        def gen_can_info(data):
            return CdsData(
                transmissionState = data[0],
                steeringWheelAngle = data[1],
                brakePedalStatus = data[2],
                brakeAppliedStatus = data[3],
                tractionControlStatus = data[4],
                antiLockBrakeStatus = data[5],
                stabilityControlStatus = data[6],
                brakeBoostApplied = data[7],
                auxiliaryBrakesStatus = data[8],
                eventHazardLights = data[9],
                eventABSactivated = data[10],
                eventTractionControlLoss = data[11],
                eventStabilityControlactivated = data[12],
                eventHardBraking = data[13],
                eventFlatTire = data[14],
                eventDisabledVehicle = data[15],
                eventAirBagDeployment = data[16],
                eventWipersStatus = data[17],
                exteriorLights = data[18],
                vehicleSpeed = data[19],
                enduranceMileage = data[20],
            )
            
        log.info(f"STEP1：IPC注入can数据")
        cds_can_data = gen_can_info(src_can_data)
        
        com_client.send_once(cds_can_data)
        
        log.info(f"STEP2: 检查CDS Proxy输出，应上报含‘Data‘的json数据：XDS Client接收的‘Data‘值 = 发送的‘Data’值")
        data = cds_client.recv_once()
        assert data != None
        assert data["Data"] == base64.b64encode(bytearray(cds_can_data)).decode()
            

   


        




