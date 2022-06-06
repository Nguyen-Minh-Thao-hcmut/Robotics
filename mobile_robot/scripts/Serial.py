#!/usr/bin/env python3
from math import modf
import rospy
from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
from mobile_robot.msg import robot_vel
import serial
L_ERROR_EST_FACTOR =    1.0
DELAY_TIME = 200
L = 0.32 * L_ERROR_EST_FACTOR
PHI = L*0.5
delta_T = 0.5


STX = 0x02
ETX = 0x03
ACK = 0x06
NAK = 0x15
SYN = 0x16
NUL = 0x00

class uart:
    FRAMESIZE = 12
    DATASIZE = 6
    def __init__(self, port_name = "/dev/ttyAMA0", baud = 115200, frame_size = FRAMESIZE, data_size = DATASIZE):
        self.port = port_name
        self.baudrate = baud
        self.txflag = 0x0
        self.rxflag = 0x0
        self.txbuffer_index = 0
        self.txcmd = b'NON'
        self.txdata = bytearray(data_size)
        self.txbuff = bytearray(frame_size)
        self.rxbuff = bytearray(frame_size)
        self.rxcmd = b"NON"
        self.rxdata = bytearray(data_size)
        self.rx_fl_ctrl = NUL
        self.rxdata_avail = 0x0
        self.t = 0
        
    
        self.connection = serial.Serial(port=self.port, baudrate=self.baudrate,parity= serial.PARITY_NONE, stopbits= serial.STOPBITS_ONE, bytesize= serial.EIGHTBITS, timeout=0.5)
        self.connection.open()
        counter = 0
        while not self.connection.is_open:
            counter += 1
            print("Serial connection fail, try to connect again")
            self.connection.open()
            if counter == 6:
                raise ConnectionError("Cannot connect to serial port, check wiring")           
        
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.serial_callback)
        self.pub = rospy.Publisher("/robot_vel", robot_vel, queue_size=5)
        print("Node init success")

    def serial_callback(self, msg):
        self.txbuffer_index = 0
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        v_L = linear_x - angular_z*PHI
        v_R = linear_x + angular_z*PHI
        list_num = []
        seperated_num = list(modf(v_L))
        list_num.append(int(seperated_num[0] * (10**2)))
        list_num.append(int(seperated_num[1]))
        seperated_num = list(modf(v_R))
        list_num.append(int(seperated_num[0] * (10**2)))
        list_num.append(int(seperated_num[1]))
        (check1, check2) = self.frame_check(bytearray(list_num))
        list_num.append(check1)
        list_num.append(check2)
        self.txdata = bytearray(list_num)
        self.txcmd = b"AUT"
        self.pack_frame(ctrl_char=SYN)
        
    def pack_frame(self, ctrl_char = SYN):
        if self.txflag == 0x0:
            self.txbuff[self.txbuffer_index] = STX
            self.txbuffer_index += 1
            self.txbuff[self.txbuffer_index: self.txbuffer_index + 3] = self.txcmd
            self.txbuffer_index += 3
            self.txbuff[self.txbuffer_index: self.txbuffer_index + len(self.txdata)] = self.txdata
            self.txbuffer_index += len(self.txdata)
            self.txbuff[self.txbuffer_index] = ctrl_char
            self.txbuffer_index += 1
            self.txbuff[self.txbuffer_index] = ETX

            self.txflag = 0x01
            #print("Pack frame success")
        else:
            print("Cannot transmit data now, please wait")


    def unpack_frame(self):
        if self.rxflag == 0x01:
            self.rx_fl_ctrl = self.rxbuff[8]
            self.rxcmd = self.rxbuff[1:4]
            self.rxdata = self.rxbuff[4:4+self.__class__.DATASIZE]
            (check1, check2) = self.frame_check(self.rxdata)
            if ((check1 == self.rxdata[5]) and (check2 == self.rxdata[6])):
                self.rxdata_avail = 0x0
            if not (self.rx_fl_ctrl == NAK):
                self.rxdata_avail = 0x01
            else:
                print("Client cannot receive data")
        self.rxflag == 0x00

    def frame_check(self, frame_byte_array):
        check1, check2 = 0
        for Byte in frame_byte_array[0:1]:
            check1 += Byte 
            check1 = check1 & 0xff
        for Byte in frame_byte_array[2:3]:
            check2 += Byte
            check2 = check2 & 0xff
            return (bytes(check1), bytes(check2))
         


