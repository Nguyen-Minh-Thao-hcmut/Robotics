from xmlrpc.client import Transport
from Serial import STX, uart,L
from geometry_msgs.msg import Twist
import rospy
from mobile_robot.msg import robot_vel
import threading

def serial_tx(connection):
    sub = rospy.Subscriber("/cmd_vel", Twist, connection.serial_callback)
    while not rospy.is_shutdown:
        if connection.txflag == 0x01:
            print("Transmitting")
            counter = 0
            while not connection.write(connection.txbuff):
                print("cannot transmit data, retransmitting")
                counter += 1
                if counter == 6:
                    raise ConnectionError("Transmittion fail, check wiring")
            print("transmittion complete")
            connection.txflag = 0x0

def serial_rx(connection):
    pub = rospy.Publisher("/robot_vel", robot_vel, queue_size=5)
    rate = rospy.Rate(77)
    while not rospy.is_shutdown():
        rxbuff_tmp = bytearray[connection._class_.FRAMESIZE]
        rxbuff_tmp = connection.connection.read(connection._class_.FRAMESIZE)
        if not rxbuff_tmp == b'':
            connection.rxbuff = rxbuff_tmp
            if (connection.rxbuff[0]==STX) and (connection.rxbuff[connection._class_.FRAMESIZE-1]):
                connection.unpack_frame()
            if connection.rxdata_avail == 0x01:
                v_R = connection.rxdata[0] + connection.rxdata[1]/100
                v_L = connection.rxdata[2] + connection.rxdata[3]/100
                connection.t = rospy.get_time()
                mess = robot_vel()
                mess.time = connection.t
                mess.vx = (v_R + v_L)/2.00
                mess.wz = (v_R - v_L)/L



def main():
    rospy.init_node("pi_stm_uart", anonymous=True)

    port = rospy.get_param("~port","/dev/ttyAMA0") # Choose first PL011 UART as default
    baudrate = int(rospy.get_param("~baud","115200"))
    ser = uart(port, baudrate)

    thread1 = threading.Thread(target=serial_rx, args=ser)
    thread2 = threading.Thread(target=serial_tx, ard = ser)

    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
