import struct
import sys
from pymavlink import mavutil
from pymavlink.DFReader import DFMessage, DFFormat
import os
from argparse import ArgumentParser

parser = ArgumentParser(description=__doc__)

parser.add_argument('this_log', metavar='this_log', type=str, help='')
parser.add_argument('ap_log', type=str, help="")
args = parser.parse_args()

SUPPORTED_VERSION = 0x01


def yxz_to_xymz(accy, accx, accz):
    return accx, accy, -accz


class ImuRaw:

    def __init__(self, sample_time, accy, accx, accz, gyry, gyrx, gyrz):
        self.sample_time = sample_time
        self.acc = [accy, accx, accz]
        self.gyr = [gyry, gyrx, gyrz]

    def get_gyr_ap(self):
        return yxz_to_xymz(*self.gyr)

    def get_acc_ap(self):
        return yxz_to_xymz(*self.acc)


class INS:

    def __init__(self, sample_time, accy, accx, accz, gyry, gyrx, gyrz, roll,
                 pitch, yaw):
        self.sample_time = sample_time
        self.att = [roll, pitch, yaw]
        self.acc = [accy, accx, accz]
        self.gyr = [gyry, gyrx, gyrz]

    def get_gyr_ap(self):
        return yxz_to_xymz(*self.gyr)

    def get_acc_ap(self):
        return yxz_to_xymz(*self.acc)


fhandle = open(args.this_log, mode="rb")
buffer = fhandle.read1(1)
(version, ) = struct.unpack("<B", buffer)
if version != SUPPORTED_VERSION:
    print("unsupported version", file=sys.stderr)
    exit(-1)

buffer = fhandle.read1(7)
if buffer != b'41moLog':
    print("wrong header ", buffer, file=sys.stderr)
    exit(-1)

ap_log = bytearray()

ATT_MSG_FMT = DFFormat(
    68,
    'ATT',
    28,
    'QccccCCCCB',
    'TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw,AEKF',
)

ACC_MSG_FMT = DFFormat(160, 'ACC', 32, 'QBQfff',
                       'TimeUS,I,SampleUS,AccX,AccY,AccZ')

GYR_MSG_FMT = DFFormat(161, 'GYR', 32, 'QBQfff',
                       'TimeUS,I,SampleUS,GyrX,GyrY,GyrZ')

IMU_MSG_FMT = DFFormat(
    162, 'IMU', 54, 'QBffffffIIfBBHH',
    'TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T,GH,AH,GHz,AHz')

FMT_MSG_FMT = DFFormat(128, 'FMT', 89, 'BBnNZ',
                       'Type,Length,Name,Format,Columns')

MODE_MSG_FMT = DFFormat(112, "MODE", 14, "QMBB", "TimeUS,Mode,ModeNum,Rsn")

PARM_MSG_FMT = DFFormat(32, "PARM", 32, "QNf", "TimeUS,Name,Value")

hdr_msg_fmt = DFMessage(
    FMT_MSG_FMT, [128, 89, 'FMT', 'BBnNZ', 'Type,Length,Name,Format,Columns'],
    False, None)

hdr_msg_imu = DFMessage(FMT_MSG_FMT, [
    162, 54, 'IMU', 'QBffffffIIfBBHH',
    'TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T,GH,AH,GHz,AHz'
], False, None)

hdr_msg_att = DFMessage(FMT_MSG_FMT, [
    68, 28, 'ATT', 'QccccCCCCB',
    'TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw,AEKF'
], False, None)

hdr_msg_acc = DFMessage(
    FMT_MSG_FMT,
    [160, 32, 'ACC', 'QBQfff', 'TimeUS,I,SampleUS,AccX,AccY,AccZ'], False,
    None)

hdr_msg_gyr = DFMessage(
    FMT_MSG_FMT,
    [161, 32, 'GYR', 'QBQfff', 'TimeUS,I,SampleUS,GyrX,GyrY,GyrZ'], False,
    None)

ap_log += hdr_msg_fmt.get_msgbuf()
ap_log += hdr_msg_imu.get_msgbuf()
ap_log += hdr_msg_att.get_msgbuf()
ap_log += hdr_msg_acc.get_msgbuf()
ap_log += hdr_msg_gyr.get_msgbuf()

ap_log += DFMessage(PARM_MSG_FMT, [0, "FORMAT_VERSION", 13], False,
                    None).get_msgbuf()

ap_log_fhandle = open(args.ap_log, mode="wb")

while True:
    buf = fhandle.read1(2)
    if buf == b'':
        break
    (_type, ) = struct.unpack("<H", buf)
    if _type == 0x01:
        buf = fhandle.read1(32)

        try:
            imu = ImuRaw(*struct.unpack("<Q" + "f" * 3 + "f" * 3, buf))
        except Exception as e:
            print("failed to parse ImuRaw ", e)
            continue

        msg1 = DFMessage(ACC_MSG_FMT,
                         [imu.sample_time, 0, 0, *imu.get_acc_ap()], False,
                         None)
        msg2 = DFMessage(GYR_MSG_FMT,
                         [imu.sample_time, 0, 0, *imu.get_gyr_ap()], False,
                         None)
        try:
            ap_log += msg1.get_msgbuf()
            ap_log += msg2.get_msgbuf()
        except Exception as e:
            pass
    if _type == 0x02:
        buf = fhandle.read1(44)

        try:
            ins = INS(*struct.unpack("<Q" + "f" * 3 + "f" * 3 + "f" * 3, buf))
        except Exception as e:
            print("failed to parse ins ", e)
            continue

        msg3 = DFMessage(IMU_MSG_FMT, [
            ins.sample_time, 0, *ins.get_gyr_ap(), *ins.get_acc_ap(), 0, 0, 0,
            0, 0, 0, 0
        ], False, None)
        msg4 = DFMessage(ATT_MSG_FMT, [
            ins.sample_time, 0, ins.att[0], 0, ins.att[1], 0, ins.att[2], 0, 0,
            0
        ], False, None)

        try:
            ap_log += msg3.get_msgbuf()
            ap_log += msg4.get_msgbuf()
        except Exception as e:
            pass
            # print("failed to get msgbuf ", e)

res = ap_log_fhandle.write(ap_log)
if res != len(ap_log):
    print("write err", file=sys.stderr)
    exit(-1)
