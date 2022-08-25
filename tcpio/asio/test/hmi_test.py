from itertools import count
import socket
from struct import pack, unpack
import pytest
from ctypes import *
from common import *
import logging
import select
from datetime import datetime
from collections import OrderedDict
import time
import json


class MockHmiClient:
    def __init__(self, server, port):
        self.hmi_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server = server
        self.port = port

    def connect(self):
        self.hmi_socket.connect((self.server, self.port))
        self.hmi_socket.setblocking(False)

    def recv_once(self, tag, timeout=3):
        start_time = datetime.now()
        while (datetime.now() - start_time).total_seconds() < timeout:
            ready = select.select([self.hmi_socket], [], [], 1.0)
            if ready[0]:
                recv_data = self.hmi_socket.recv(8)
                start_flag, length = unpack('>II', recv_data)
                if start_flag == 0x00FF0000 and length < 65536:
                    recv_data = self.hmi_socket.recv(length)
                    package = json.loads(recv_data)
                    if package['tag'] == tag:
                        return package
                    
    def recv_last(self, tag, timeout=3):
        ret = None
        start_time = datetime.now()
        while (datetime.now() - start_time).total_seconds() < timeout:
            package = self.recv_once(tag, timeout)
            if package:
                ret = package
        return ret
    
    def send_once(self, json_data):
        body = json.dumps(json_data).encode()
        header = pack('>II', 0x00FF0000, len(body))
        self.hmi_socket.send(header + body)
    
    def close(self):
        self.hmi_socket.close()

    def clear(self):
        while True:
            inputready, o, e = select.select([self.hmi_socket], [], [], 0)
            if len(inputready)==0: 
                break
            for s in inputready: 
                s.recv(1)



log = logging.getLogger(__name__)

target = "127.0.0.1"
com_port = 35558
hmi_port = 35553

com_client = ComClient(target, com_port)
hmi_client = MockHmiClient(target, hmi_port)

req_success_test_data = [
    # subject, req_tag, resp_tag, m_req_field, m_req_expected_value, m_resp_field, m_resp_value
    ("CACC ON",  3001,    3002, "De_CACCSWOn_u8", True,      "De_CACC_CACCOn_Rsp_u8",  1), 
    ("CACC OFF", 3003,    3004, "De_CACCSWOn_u8", False,     "De_CACC_CACCOff_Rsp_u8", 1), 
]

req_resp_with_error_test_data = [
    # subject, req_tag, resp_tag, m_resp_field, m_resp_value, expected_rsp
    ("CACC ON", 3001,     3002, "De_CACC_CACCOn_Rsp_u8", 0,   -3), 
    ("CACC ON", 3001,     3002, "De_CACC_CACCOn_Rsp_u8", 5,    5), 
]

req_resp_timeout_server_busy_test_data = [
    ("CACC ON", 3001, 3002), 
]
        

@pytest.fixture(scope="session", autouse=True)
def entry():
    com_client.connect()
    hmi_client.connect()

    yield

    com_client.close()
    hmi_client.close()


class TestClasshmiProxy:

    def test_hmi_cacc_status(self):
        """HMI_TEST_001 注入CACC 状态数据，观察hmi客户端接收的数据是否和预期一致"""
        def gen_hmi_info(acc_status, ida_status):
            return HmiInfo(De_CACC_CACCStatus_u8 = acc_status, De_CACC_IDAStatus_u8=ida_status)

        log.info(f"STEP1: 注入CACC状态改变")
        hmi_info = gen_hmi_info(0, 0)
        com_client.send_once(hmi_info)
        time.sleep(0.05)
        hmi_info = gen_hmi_info(1, 0)
        com_client.send_once(hmi_info)

        log.info(f"STEP2: 检查HMI Proxy输出，应上报2001数据")
        data = hmi_client.recv_last(2001)
        assert data != None
        assert data["data"]["ACCStatus"] == 1

        log.info(f"STEP3: 注入IDA状态改变")
        hmi_info = gen_hmi_info(1, 1)
        com_client.send_once(hmi_info)

        log.info(f"STEP4: 检查HMI Proxy输出，应上报2001数据")
        data = hmi_client.recv_once(2001)
        assert data != None
        assert data["data"]["IDAStatus"] == 1

        log.info(f"STEP5: 注入相同数据，即不引起状态改变")
        hmi_info = gen_hmi_info(1, 1)
        com_client.send_once(hmi_info)

        log.info(f"STEP6: HMI Proxy应不上报2001数据")
        data = hmi_client.recv_once(2001)
        assert data == None

    def test_hmi_caeb_status(self):
        """HMI_TEST_002 注入CAEB 状态数据，观察hmi客户端接收的数据是否和预期一致"""
        def gen_hmi_info(aeb_status):
            return HmiInfo(
                De_FCW_AEB_FuncSts_u8 = aeb_status,
            )

        log.info(f"STEP1: 注入CAEB状态改变")
        hmi_info = gen_hmi_info(0)
        com_client.send_once(hmi_info)
        time.sleep(0.05)
        hmi_info = gen_hmi_info(2)
        com_client.send_once(hmi_info)

        log.info(f"STEP2: 检查HMI Proxy输出，应上报2002数据")
        data = hmi_client.recv_last(2002)
        log.info(data)
        assert data != None
        assert data["data"]["CAEBStatus"] == 2

        log.info(f"STEP3: 注入相同数据，即不引起状态改变")
        hmi_info = gen_hmi_info(2)
        com_client.send_once(hmi_info)

        log.info(f"STEP4: HMI Proxy应不上报2002数据")
        data = hmi_client.recv_once(2002)
        assert data == None

    def test_hmi_system_error(self):
        """HMI_TEST_003 注入system error数据，观察hmi客户端接收的数据是否和预期一致"""

        log.info(f"STEP1: send error message with error 1 and 2 active")
        msg_type = MsgType.IPC_SYS_ERROR
        error = [0] * 256
        error[0] = 1
        error[1] = 2
        start_time = datetime.now()
        while (datetime.now() - start_time).total_seconds() < 3:
            com_client.send_once(bytearray(error), msg_type)
            time.sleep(0.05)

        data = hmi_client.recv_once(2003)
        log.info(data)

        log.info(f"STEP2: check the response, only error 1 and 2 shall be presented")
        system_error = data["data"]["systemError"]
        assert len(system_error) == 2
        assert system_error[0]["type"] == 1
        assert system_error[1]["type"] == 2
        
        log.info(f"STEP3: send error message with error 1 recovered and 2 active")
        error = [0] * 256
        error[0] = 2
        com_client.send_once(bytearray(error), msg_type)

        log.info(f"STEP4: check the response, only error 2 shall be presented")
        data = hmi_client.recv_last(2003)
        log.info(data)
        system_error = data["data"]["systemError"]
        assert len(system_error) == 1
        assert system_error[0]["type"] == 2
    
    def test_hmi_vehicle_info(self):
        """HMI_TEST_004 发送车辆信息，观察hmi客户端接收的数据是否和预期一致""" 
        heading = GNSS_HeadingPitchRoll(heading=2.859)
        lat_long = GNSS_LatitudeLongitude(11.5, 12.1)
        evh = EvhInfo(De_ego_vxMs_f32=52.8, De_ego_axMs2_f32=3, De_ego_ayMs2_f32=4)
        com_client.send_once(heading)
        com_client.send_once(lat_long)
        com_client.send_once(evh)

        start_time = datetime.now()
        msg_timestamp = []
        last_msg = None
        time.sleep(0.050)
        hmi_client.clear()
        while (datetime.now() - start_time).total_seconds() < 10:
            data = hmi_client.recv_once(2004)
            if data:
                msg_timestamp.append(datetime.now())
                last_msg = data

        deviation = [abs((b - a).total_seconds() - 0.2) for a, b in zip(msg_timestamp[:-1], msg_timestamp[1:])]
        devication_prc = max(deviation)/0.1
        log.info(f"max cycle time deviation is {max(deviation)*1000} ms, {devication_prc*100}%")
        assert last_msg != None
        assert last_msg['data']['acceleration'] == 5
        assert last_msg['data']['latitude'] == pytest.approx(11.5, 0.000001)
        assert last_msg['data']['longitude'] == pytest.approx(12.1, 0.000001)
        assert last_msg['data']['heading'] == pytest.approx(2.86, 0.001)
        assert last_msg['data']['speed'] == 52.8


    @pytest.mark.parametrize("subject, req_tag, resp_tag, m_req_field, m_req_expected_value, m_resp_field, m_resp_value", req_success_test_data)
    def test_hmi_request_success(self, subject, req_tag, resp_tag, m_req_field, m_req_expected_value, m_resp_field, m_resp_value):
        """HMI_TEST_008 发送请求, M核响应设置成功时，HMI Client应能正确接受到响应""" 

        def gen_hmi_info(field, value, counter):
            data = {field: value, "De_ResponseCounter_u32": counter}
            return HmiInfo(**data)


        log.info(f"STEP1: 发送{subject}请求，M核响应请求(Positive Response)")
        hmi_client.send_once({"tag":req_tag})

        log.info(f"STEP2: 检查核间通信接口，M核应能正确接收到{subject}请求，并回复请求成功")
        
        hmi_ctrl = com_client.recv_once(MsgType.IPC_HMI_CTRL)
        assert hmi_ctrl != None
        assert getattr(hmi_ctrl, m_req_field) == m_req_expected_value
        hmi_info = gen_hmi_info(m_resp_field, m_resp_value, hmi_ctrl.De_RequestCounter_u32 + 1)
        com_client.send_once(hmi_info)

        log.info(f"STEP3: 检查HMI Proxy输出，此时应返回rsp==0,detail==''")
        data = hmi_client.recv_once(resp_tag)
        assert data != None
        assert data["rsp"] == 0
        assert data["detail"] == ""

    @pytest.mark.parametrize("subject, req_tag, resp_tag, m_resp_field, m_resp_value, expected_rsp", req_resp_with_error_test_data)
    def test_hmi_request_resp_with_error(self, subject, req_tag, resp_tag, m_resp_field, m_resp_value, expected_rsp):
        """HMI_TEST_008 发送请求, M核相应设置失败时，HMI Client应能正确接受到响应，rsp==M核响应""" 

        def gen_hmi_info(field, value, counter):
            data = {field: value, "De_ResponseCounter_u32": counter}
            return HmiInfo(**data)


        log.info(f"STEP1: 发送{subject}请求，M核响应请求(Negative Response)")
        hmi_client.send_once({"tag":req_tag})

        hmi_ctrl = com_client.recv_once(MsgType.IPC_HMI_CTRL)
        hmi_info = gen_hmi_info(m_resp_field, m_resp_value, hmi_ctrl.De_RequestCounter_u32 + 1)
        com_client.send_once(hmi_info)

        log.info(f"STEP2: 检查HMI Proxy输出，此时应返回rsp=={m_resp_value}")
        data = hmi_client.recv_once(resp_tag)
        assert data != None
        assert data["rsp"] == expected_rsp

    @pytest.mark.parametrize("subject, req_tag, resp_tag", req_resp_timeout_server_busy_test_data)
    def test_hmi_request_no_resp_from_m(self, subject, req_tag, resp_tag):
        """HMI_TEST_008 发送请求, M核不响应，HMI Client应能正确接受到响应，rsp==-1""" 

        log.info(f"STEP1: 发送{subject}请求，M核响应丢失")
        hmi_client.send_once({"tag":req_tag})
        
        log.info(f"STEP2: 检查HMI Proxy输出，此时应返回{resp_tag}， rsp:-1")
        data = hmi_client.recv_once(resp_tag)
        assert data != None
        assert data["rsp"] == -1

    @pytest.mark.parametrize("subject, req_tag, resp_tag", req_resp_timeout_server_busy_test_data)
    def test_hmi_request_old_resp_from_m(self, subject, req_tag, resp_tag):
        """M核响应上次的请求时，HMI Client应能接受到响应，rsp==-2""" 
        def gen_hmi_info(counter):
            data = {"De_ResponseCounter_u32": counter}
            return HmiInfo(**data)

        log.info(f"STEP1: 发送{subject}请求，M核回应上次请求")
        hmi_client.send_once({"tag":req_tag})

        hmi_ctrl = com_client.recv_once(MsgType.IPC_HMI_CTRL)
        assert hmi_ctrl != None
        hmi_info = gen_hmi_info(hmi_ctrl.De_RequestCounter_u32)
        com_client.send_once(hmi_info)

        log.info(f"STEP2: 检查HMI Proxy输出，此时应返回{resp_tag}, rsp:-2")
        data = hmi_client.recv_once(resp_tag)
        assert data != None
        assert data["rsp"] == -2
        
    @pytest.mark.parametrize("subject, req_tag, resp_tag", req_resp_timeout_server_busy_test_data)
    def test_hmi_request_server_busy(self, subject, req_tag, resp_tag):
        """PAD重复发送请求，应不对上次请求进行响应，且本次响应rsp==-4""" 

        log.info(f"STEP1: 发送{subject}请求")
        hmi_client.send_once({"tag":req_tag})
        
        log.info(f"STEP2: 再次发送{subject}请求")
        hmi_client.send_once({"tag":req_tag})
        
        log.info(f"STEP3: 检查HMI Proxy输出，此时应返回{resp_tag}, rsp:-4")
        data = hmi_client.recv_once(resp_tag)
        assert data != None
        assert data["rsp"] == -4

        log.info(f"STEP4: 检查HMI Proxy输出，此时应无多余的响应")
        data = hmi_client.recv_once(resp_tag)
        assert data == None

    def test_hmi_connect(self):
        '''模拟PAD向HMI Proxy发送请求，应可以正常接收到'''
        log.info(f"STEP1: PAD发送HMI建立连接请求，PAD应可以接收到0004响应")
        hmi_client.send_once({"tag":3, "data":{"clientIp":"127.0.0.1", "clientPort":9001, "deviceSerialNum": "CDC1X32"}})
        
        log.info(f"STEP2: 检查HMI Proxy输出，此时应返回0004响应")
        data = hmi_client.recv_once(4)
        assert data != None
        assert data["rsp"] == 0
        assert data["data"]['deviceSerialNum'] == "CDC1X32"
        
        log.info(f"STEP2: 检查HMI Proxy输出，此时应上报2007系统开关状态")
        data = hmi_client.recv_once(2007)
        assert data != None



