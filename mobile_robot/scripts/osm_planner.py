#!/usr/bin/python3

import numpy as np
import tf as ros_tf
import utm

import rospy
import rospkg
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

from pyroutelib3 import Router

rospack = rospkg.RosPack()
router = Router("foot",rospack.get_path('my_robot') + '/maps/hcmut.osm')

rospy.init_node("OSM_Planner")
tf_listener = ros_tf.TransformListener()    

busy_executing_plan = False
output_map_path_msg = Path()

   
def find_route(pos, goal):
    routeLatLons = None

    start = router.findNode(pos['lat'], pos['lon']) # Find start and end nodes
    end = router.findNode(goal['lat'], goal['lon'])

    status, route = router.doRoute(start, end) # Find the route - a list of OSM nodes
    if status == 'success':
        routeLatLons = list(map(router.nodeLatLon, route)) # Get actual route coordinates
    return routeLatLons
       
    
def generate_path_msg(waypoints, headings):
    path_msg = Path()
    path_msg.header = Header()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = 'utm'
    
    for i in range(len(waypoints)):
        single_point_pose = PoseStamped()
        single_point_pose.header = Header()
        single_point_pose.header.stamp = rospy.Time.now()
        single_point_pose.header.frame_id = 'utm'
        single_point_pose.pose = Pose(Point(waypoints[i][0], waypoints[i][1], 0.), Quaternion(*headings[i]))
        path_msg.poses.append(single_point_pose)
    
    return path_msg
    
    
def calculate_path(pos, goal):
    routeLatLons = find_route(pos, goal)
    
    utm_waypoints = []
    for (lat, lon) in routeLatLons:
        x, y, _, _ = utm.from_latlon(lat, lon)
        utm_waypoints.append((x, y))
        
    headings = []
    for i in range(len(utm_waypoints)):
        if i == 0:
            try:
                tf_listener.waitForTransform("base_link", "utm", rospy.Time(), rospy.Duration(5.0))
            except (ros_tf.LookupException):
                rospy.logerr('Cannot find utm->base_link tf')
                return None 
            (trans, rot) = tf_listener.lookupTransform('base_link', 'utm', rospy.Time(0))
            headings.append(rot)
        else:
            yaw = np.arctan2(utm_waypoints[i][1] - utm_waypoints[i-1][1], utm_waypoints[i][0] - utm_waypoints[i-1][0])
            quat = ros_tf.transformations.quaternion_from_euler(0, 0, yaw)
            headings.append(quat)
    
    return generate_path_msg(utm_waypoints, headings)
    
def move_base_stt_callback(msg):
    # msg type: actionlib_msgs/GoalID
    global busy_executing_plan
    try:
        moving_status = msg.status_list[0].status
        # moving_status == 1: executing plan
        # moving_status == 2: plan canceled
        # moving_status == 3: reach goal
        if (moving_status == 2) or (moving_status==3):
            busy_executing_plan = False
        if (moving_status == 1):
            busy_executing_plan = False
    except:
        pass   
    

def map_goal_callback(msg):
    global geo_goal_pub
    global busy_executing_plan

    map_goal = msg.goal.target_pose

    t = tf_listener.getLatestCommonTime("utm", "map")
    map_goal.header.frame_id = "map"
    utm_goal = tf_listener.transformPose("utm", map_goal)
    lat, lon = utm.to_latlon(utm_goal.pose.position.x, utm_goal.pose.position.y, 48, 'P')
    
    geo_goal_msg = NavSatFix()
    geo_goal_msg.header = Header()
    geo_goal_msg.header.stamp  = rospy.Time.now()

    geo_goal_msg.latitude  = lat
    geo_goal_msg.longitude = lon
    geo_goal_pub.publish(geo_goal_msg)
    
    
def gps_coordinate_callback(msg):
    # msg type: sensor_msgs/NavSatFix
    global gps_position
    gps_position = {'lat': msg.latitude, 'lon': msg.longitude}
    

def geo_goal_callback(msg):
    # msg type: sensor_msgs/NavSatFix   
    global output_map_path_msg
    global busy_executing_plan
    global gps_position

    if busy_executing_plan == True or busy_executing_plan == None:
        rospy.logerr('Cancel last move_base command before execute new one!')
    if busy_executing_plan == False:
        rospy.loginfo('New nav_goal received')
        geo_goal = {'lat': msg.latitude, 'lon': msg.longitude}
        output_map_path_msg = calculate_path(gps_position, geo_goal)
	
	
if __name__ == '__main__':
    move_base_stt_sub = rospy.Subscriber('move_base/status', GoalStatusArray, move_base_stt_callback, queue_size=1)
    gps_sub = rospy.Subscriber('gps/fix', NavSatFix, gps_coordinate_callback, queue_size=1)
    geo_goal_sub = rospy.Subscriber('move_base/geo_goal', NavSatFix, geo_goal_callback, queue_size=1)
    map_goal_sub = rospy.Subscriber('move_base/goal', MoveBaseActionGoal, map_goal_callback, queue_size=1)
    path_pub = rospy.Publisher('move_base/OSMPlanner/plan', Path, queue_size=10)
    geo_goal_pub = rospy.Publisher('move_base/geo_goal', NavSatFix, queue_size=1)
    
    rate = rospy.Rate(5) 
    
    while not rospy.is_shutdown():
        try:
            path_pub.publish(output_map_path_msg)
        except rospy.ROSInterruptException:
            pass
        rate.sleep()

