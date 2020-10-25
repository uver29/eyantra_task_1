#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

import math

pose = [0, 0, 0]


def Waypoints(t):
    x = t + 0.1
    y = 2 * math.sin(x) * math.sin(x / 2)
    return [x, y]


def control_loop():
    rospy.init_node('ebot_controller')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10)

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    while not rospy.is_shutdown():

        #
        # Your algorithm to complete the obstacle course
        #

        goal = Waypoints(pose[0])
        theta_goal = math.atan((goal[1] - pose[1]) / (goal[0] - pose[0]))
        e_theta = theta_goal - pose[2]

        velocity_msg.linear.x = 0.6
        velocity_msg.angular.z = 3.5 * e_theta
        pub.publish(velocity_msg)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()


def odom_callback(data):
    global pose
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y,
            euler_from_quaternion([x, y, z, w])[2]]


def laser_callback(msg):
    global regions
    regions = {
        # 'bright':     ,
        # 'fright':     ,
        # 'front':      ,
        # 'fleft':      ,
        # 'bleft':      ,
    }


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
