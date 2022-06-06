#!/usr/bin/python3
import rospy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
import tf
ekf_quat = Quaternion()
def ekf_callback(msg):
    global ekf_quat
    ekf_quat = msg.pose.pose.orientation
def main():
    r = rospy.Rate(50)
    global ekf_quat
    while not rospy.is_shutdown():
        ekf_sub = rospy.Subscriber("/odometry/filtered", Odometry, ekf_callback)
        ekf_pub = rospy.Publisher("/yaw/filtered", Float32, queue_size=10)

        quaternion = [ekf_quat.x, ekf_quat.y, ekf_quat.z, ekf_quat.w]
        (_,_,yaw) = tf.transformations.euler_from_quaternion(quaternion)
        ekf_pub.publish(yaw)
        r.sleep()
if __name__ == "__main__":
    main()
