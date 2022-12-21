import socket
import pytest

from common import *
import logging
import select
from datetime import datetime
from collections import OrderedDict


log = logging.getLogger(__name__)

target = "127.0.0.1"
com_port = 35558
xds_port = 35559


xds_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
com_client = ComClient(target, com_port)

@pytest.fixture(scope="session", autouse=True)
def connect():
    com_client.connect()
    xds_client.connect((target, xds_port))
    xds_client.setblocking(False)

    yield

    com_client.close()
    xds_client.close()


class TestClassXdsProxy:

    def test_GNSS_ACC(self):
        """注入GNSS ACC数据，观察XDS客户端接收的数据是否和预期一致"""
        expected_XYZ = '$GNSSINS570,XYZ,2.100000,3.100000,1.100000,123.456703,123.456703,123.456703,234.567001,-234.567001,234.567001,80.000000'
        log.info(f"EXPECTED: {expected_XYZ}") 
        expected_GPS = '$GNSSINS570,GPS,16,4,18,1,1,2'
        log.info(f"EXPECTED: {expected_GPS}")
        expected_STD = '$GNSSINS570,STD,100.134003,-50.456001,100.134003,0.200000'
        log.info(f"EXPECTED: {expected_STD}") 
        expected_UTC = '$GNSSINS570,UTC,22,7,5,16,30,25,680'
        log.info(f"EXPECTED: {expected_STD}")

        ins_acc = GNSS_Acc(2.1, 3.1, 1.1)
        log.info(f"STEP1: send gnss acc message: {ins_acc}")
        com_client.send_once(ins_acc)

        ins_gyro = GNSS_Gyro(123.456703, 123.456703, 123.456703)
        log.info(f"STEP2: send gnss gyro message: {ins_gyro}")
        com_client.send_once(ins_gyro)

        ins_heading = GNSS_HeadingPitchRoll(234.567001, -234.567001, 234.567001)
        log.info(f"STEP3: send gnss heading message: {ins_heading}")
        com_client.send_once(ins_heading)

        ins_height = GNSS_HeightAndTime(234.12345, 234)
        log.info(f"STEP4: send gnss height message: {ins_height}")
        com_client.send_once(ins_height)

        ins_lat = GNSS_LatitudeLongitude(100.134, -50.456)
        log.info(f"STEP5: send gnss lat message: {ins_lat}")
        com_client.send_once(ins_lat)

        ins_speed = GNSS_Speed(100.134, -50.456, 134.567)
        log.info(f"STEP6: send gnss speed message: {ins_speed}")
        com_client.send_once(ins_speed)

        ins_data = GNSS_DataInfo(16, 4, 18, 1, 1, 2, 1)
        log.info(f"STEP7: send gnss data message: {ins_data}")
        com_client.send_once(ins_data)

        ins_std = GNSS_Std(100.134, -50.456, 200.3, 0.2)
        log.info(f"STEP8: send gnss std message: {ins_std}")
        com_client.send_once(ins_std)

        ins_utc = GNSS_UTC(22, 7, 5, 16, 30, 25, 680, 1)
        log.info(f"STEP9: send gnss utc message: {ins_utc}")
        com_client.send_once(ins_utc)

        log.info(f"STEP10: check received xds message format")
        start_time = datetime.now()
        recv_data = ''
        xyz_timestamp_list = []
        while (datetime.now() - start_time).total_seconds() < 3:
            ready = select.select([xds_client], [], [], 2.0)
            if ready[0]:
                recv_data += xds_client.recv(4096).decode('ascii')
                log.info(f"STEP11: check received xds message: {recv_data}")
                for line in recv_data.split("\r\n"):
                    if "XYZ" in line:
                        xyz_timestamp_list.append(datetime.now())
                        assert line == expected_XYZ
                    elif "GPS" in line:
                        assert line == expected_GPS
                    elif "STD" in line:
                        assert line == expected_STD
                    elif "UTC" in line:
                        assert line == expected_UTC
                recv_data = recv_data[recv_data.rindex('\r\n') + 2:]

        log.info(f"STEP12: check received xds message cycle time")
        deviation = [abs((b - a).total_seconds() - 0.1) for a, b in zip(xyz_timestamp_list[:-1], xyz_timestamp_list[1:])]
        devication_prc = max(deviation)/0.1
        log.info(f"max cycle time deviation is {max(deviation)*1000} ms, {devication_prc*100}%")
        assert devication_prc < 0.1



