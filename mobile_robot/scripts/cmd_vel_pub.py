#!/usr/bin/python3

import rospy
import math
import tf

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, NavSatFix

cnt = 0
vel0 = 0
vel1 = 0.03
vel2 = 0.02
vel3 = 0.05

def main():
    rospy.init_node('Publish_cmd_vel')

    cmd_vel = Twist()
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10) # 50hz
    
    while not rospy.is_shutdown():
        global cnt, vel1, vel2, vel3, vel0
        cnt += 1
        if cnt <= 150:
            cmd_vel.linear.x = vel1
        elif cnt > 300 and cnt <= 450:
            cmd_vel.linear.x = -vel1
        elif cnt > 600 and cnt <= 750:
            cmd_vel.linear.x = vel2
        elif cnt > 900 and cnt <= 1050:
            cmd_vel.linear.x = -vel2
        elif cnt > 1200 and cnt <= 1350:
            cmd_vel.linear.x = vel3
        elif cnt > 1500 and cnt <= 1650:
            cmd_vel.linear.x = -vel3
        elif cnt == 1800:
            cnt = 0
        else:
            cmd_vel.linear.x = vel0           
            
        cmd_vel_pub.publish(cmd_vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
