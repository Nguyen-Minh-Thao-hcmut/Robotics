#!/usr/bin/python3

import rospy
import math
import tf

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, NavSatFix
quat1 = Quaternion()
global x, y, th, vx, vy, vth, dt, current_time, last_time
def vel_callback(vel_pub):
    global x, y, th, vx, vy, vth
    vx = vel_pub.linear.x
    vth = vel_pub.angular.z
    current_time = vel_pub.time
    
def theta_callback(odom):
    global x, y, th, vx, vy, vth, quat1
    quat1 = odom.pose.pose.orientation
    explicit_quat = [quat1.x, quat1.y, quat1.z, quat1.w]
    (_,_,th) = tf.transformations.euler_from_quaternion(explicit_quat)
    
def main():

    rospy.init_node('Serial_Process')
    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0
    vy = 0
    vth = 0
    odom = Odometry()
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

    rospy.Subscriber('/robot_vel', Twist, vel_callback)
    tf_broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(60)
   
    while not rospy.is_shutdown():
        
        vx1 = vx
        vth1 = vth
        
        dt = (current_time - last_time).to_sec()
        delta_th = vth1 * dt
        delta_s = vx1*dt
        delta_x = delta_s*math.cos(th + delta_th/2)
        delta_y = delta_s*math.sin(th + delta_th/2)

        x += delta_x
        y += delta_y
        th += delta_th
       
      
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
        odom.child_frame_id = 'base_link'
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))


        odom_pub.publish(odom)
        last_time = current_time
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
