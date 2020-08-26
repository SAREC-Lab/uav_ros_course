#!/usr/bin/env python
# Software License Agreement (BSD License)
# Modified from https://www.theconstructsim.com/ros-qa-053-how-to-move-a-robot-to-a-certain-point-using-twist/
# to work with TurtleBot3-Burger

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x = 0.0
y = 0.0 
theta = 0.0

# Callback function
def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

# Initialize mode
rospy.init_node("speed_controller")

# Subscribe to the odom topic to get information about the current position and velocity
# of the robot
sub = rospy.Subscriber("/odom", Odometry, newOdom)

# Publish linear and angular velocities to cmd_vel topic
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

# Establish target coordinates
goal = Point()
goal.x = 5
goal.y = 5

# Strategy is to first turn to face the target coordinates
# and then move towards them
while not rospy.is_shutdown():

    # Compute difference between current position and target position
    inc_x = goal.x -x
    inc_y = goal.y -y
    angle_to_goal = atan2(inc_y, inc_x)
    
    difference_angle = abs(angle_to_goal - theta)
    msg2 = ("Difference: %s " % difference_angle)
    rospy.loginfo(msg2)

    # Check whether to turn or move.
    if abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.13
    else:
        rospy.loginfo("Ready to move")
        speed.linear.x = 0.5
        speed.angular.z = 0.0
    
    pub.publish(speed)
    r.sleep()   