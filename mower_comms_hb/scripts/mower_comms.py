#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from mower_msgs.msg import Status, ImuRaw, ESCStatus
from mower_msgs.srv import MowerControlSrv, MowerControlSrvResponse, EmergencyStopSrv, EmergencyStopSrvResponse, GPSControlSrv

import qwiic_icm20948

import serial
import struct
import sys
import time
import crcmod.predefined
import threading
from collections import namedtuple

comms_left = serial.Serial(port='/dev/ttyAMA1', baudrate=115200, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, timeout=0.1)
comms_right = serial.Serial(port='/dev/ttyAMA2', baudrate=115200, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, timeout=0.1)

IMU = qwiic_icm20948.QwiicIcm20948()

speed_l = 0.0
speed_r = 0.0
speed_mow = 0.0
emergency_high_level = False
status_pub = None
imu_pub = None
last_imu_ts = rospy.Time()
last_cmd_vel = rospy.Time()
ll_status_mutex = threading.Lock()
ll_status = {
  'left': None,
  'right': None,
}

xmodem = crcmod.predefined.mkPredefinedCrcFun('xmodem')
MotorData = namedtuple('MotorData', [
    'ticks',
    'currentDC',
    'batteryVoltage',
    'charging',
])


def read_motor(comms):
    rx_data = comms.read(17)
    comms.reset_input_buffer()
    if len(rx_data) < 17:
        raise IOError('Short UART read')
    start, ticks, currentDC, batteryVoltage, data, crc, stop = struct.unpack('>cqHHBHc', rx_data)
    crc_exp = xmodem(rx_data[0:-3])
    if crc != crc_exp:
        raise IOError('CRC check failed')
    charging = data & (1 << 0)
    # NB: motors are reversed so negate ticks
    return MotorData(-ticks, currentDC, batteryVoltage, charging)

def write_motor(comms, mode, speed):
    data = 0
    data |= (1 << 0) # enable
    # NB: motors are reversed so negate speed
    tx_data = struct.pack('>chBHB', b'/', -speed, 0, 0, data)
    tx_data += struct.pack('>Hc', xmodem(tx_data), b'\n')
    comms.write(tx_data)
    comms.flush()

def is_emergency():
    return emergency_high_level

def publishActuators():
    global speed_l
    global speed_r
    global speed_mow
    global last_cmd_vel
    if is_emergency():
        speed_l = 0
        speed_r = 0
        speed_mow = 0
    if rospy.Time.now() - last_cmd_vel > rospy.Duration(1):
        speed_l = 0
        speed_r = 0
    if rospy.Time.now() - last_cmd_vel > rospy.Duration(25):
        speed_l = 0
        speed_r = 0
        speed_mow = 0
    try:
        write_motor(comms_left, 0, -int(speed_l*1000))
    except Exception:
        rospy.logerr("Error writing to left motor serial port", exc_info=True)
    try:
        write_motor(comms_right, 0, int(speed_r*1000))
    except Exception:
        rospy.logerr("Error writing to right motor serial port", exc_info=True)

def publishStatus():
    # TODO: should this acquire a lock in mower_comms.cpp?
    with ll_status_mutex:
        left_data = ll_status['left']
        right_data = ll_status['right']

    status_msg = Status()
    status_msg.stamp = rospy.Time.now()
    status_msg.mower_status = Status.MOWER_STATUS_OK
    status_msg.raspberry_pi_power = 1
    status_msg.gps_power = 1
    status_msg.esc_power = 1
    status_msg.rain_detected = 0
    status_msg.sound_module_available =0
    status_msg.sound_module_busy = 0
    status_msg.ui_board_available = 0
    status_msg.ultrasonic_ranges = [0,0,0,0,0]
    status_msg.emergency = is_emergency()

    l_status = ESCStatus()
    if left_data is not None:
        status_msg.v_battery = left_data.batteryVoltage/1000
        status_msg.v_charge = left_data.charging
        status_msg.charge_current = 1 if left_data.charging else 0
        l_status.status = ESCStatus.ESC_STATUS_OK
        l_status.tacho = left_data.ticks
        l_status.current = left_data.currentDC/1000
        l_status.temperature_motor = 30
        l_status.temperature_pcb = 30
    else:
        l_status.status = ESCStatus.ESC_STATUS_DISCONNECTED

    r_status = ESCStatus()
    if right_data is not None:
        r_status.status = ESCStatus.ESC_STATUS_OK
        r_status.tacho = right_data.ticks
        r_status.current = right_data.currentDC/1000
        r_status.temperature_motor = 30
        r_status.temperature_pcb = 30
    else:
        r_status.status = ESCStatus.ESC_STATUS_DISCONNECTED
    m_status = ESCStatus()
    m_status.status = ESCStatus.ESC_STATUS_OK

    status_msg.left_esc_status = l_status
    status_msg.right_esc_status = r_status
    status_msg.mow_esc_status = m_status

    status_pub.publish(status_msg)

def publishActuatorsTimerTask(*args, **kwargs):
    handleLowLevelIMU()
    publishActuators()
    publishStatus()

def setMowEnabled(req):
    global speed_mow
    if req.mow_enabled and not isEmergency():
        speed_mow = 1;
    else:
        speed_mow = 0;
    rospy.loginfo("Setting mow enabled to {}".format(speed_mow))
    return MowerControlSrvResponse()

def setEmergencyStop(req):
    global emergency_high_level
    if req.emergency:
        rospy.logerr("Setting emergency!!")
    emergency_high_level = req.emergency
    publishActuators()
    return EmergencyStopSrvResponse()

def velReceived(msg):
    global speed_l
    global speed_r
    global last_cmd_vel

    last_cmd_vel = rospy.Time.now()
    # TODO: figure out why the signs are reversed in mower_comms.cpp
    speed_l = msg.linear.x - msg.angular.z;
    speed_r = msg.linear.x + msg.angular.z;

    if speed_l >= 1.0:
        speed_l = 1.0;
    elif speed_l <= -1.0:
        speed_l = -1.0
    if speed_r >= 1.0:
        speed_r = 1.0;
    elif speed_r <= -1.0:
        speed_r = -1.0

def handleLowLevelStatus():
    global ll_status
    left_data = right_data = None
    try:
        left_data = read_motor(comms_left)
    except IOError as e:
        rospy.logwarn("Failed to read left motor status: {}".format(e))
    try:
        right_data = read_motor(comms_right)
    except IOError as e:
        rospy.logwarn("Failed to read right motor status: {}".format(e))
    with ll_status_mutex:
        ll_status['left'] = left_data
        ll_status['right'] = right_data

def handleLowLevelIMU():
    global IMU
    global imu_pub
    global last_imu_ts
    if IMU.dataReady():
        try:
            IMU.getAgmt()
        except IOError as e:
            rospy.logwarn("Failed to read ICM-20948 data: {}".format(e))
        else:
            imu_msg = ImuRaw()
            # Constants take from https://github.com/adafruit/Adafruit_CircuitPython_ICM20X
            imu_msg.ax, imu_msg.ay, imu_msg.az = (IMU.axRaw / 16384 * 9.80665, IMU.ayRaw / 16384 * 9.80665, IMU.azRaw / 16384 * 9.80665)
            imu_msg.mx, imu_msg.my, imu_msg.mz = ((IMU.mxRaw*0.15)-13.2358, (IMU.myRaw*0.15)-77.6404, IMU.mzRaw*0.15)
            imu_msg.gx, imu_msg.gy, imu_msg.gz = (IMU.gyRaw / 131.0 * 0.017453293, IMU.gxRaw / 131.0 * 0.017453293, IMU.gzRaw / 131.0 * 0.017453293)
            now = rospy.Time.now()
            imu_msg.dt = (now - last_imu_ts).to_nsec()//1000000
            last_imu_ts = now
            imu_pub.publish(imu_msg)

def main():
    rospy.init_node('mower_comms')

    global speed_l
    global speed_r
    global status_pub
    global imu_pub
    global last_imu_ts

    # TODO: use params for ports

    speed_l = speed_r = speed_mow = 0.0

    status_pub = rospy.Publisher('mower/status', Status, queue_size=1)
    imu_pub = rospy.Publisher('mower/imu', ImuRaw, queue_size=1)
    mow_service = rospy.Service("mower_service/mow_enabled", MowerControlSrv, setMowEnabled)
    emergency_service = rospy.Service("mower_service/emergency", EmergencyStopSrv, setEmergencyStop)
    cmd_vel_sub = rospy.Subscriber("cmd_vel",Twist, velReceived, tcp_nodelay=True)

    while IMU.connected == False:
        rospy.logwarn("ICM-20948 not connected")
    IMU.begin()
    last_imu_ts = rospy.Time.now()

    publish_timer = rospy.timer.Timer(rospy.Duration(0.02), publishActuatorsTimerTask)

    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        handleLowLevelStatus()
        # handleLowLevelIMU moved to publishActuatorsTimerTask to mimic the real IMU messages rate of every 20ms
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
         pass
