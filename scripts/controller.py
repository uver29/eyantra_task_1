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

        # Following the path: 2(sin(x))(sin(x/2)) from x = 0 to 2pi
        while pose[0] < 2 * math.pi:
            waypoint_goal = Waypoints(pose[0])
            theta_goal = math.atan2(
                (waypoint_goal[1] - pose[1]), (waypoint_goal[0] - pose[0]))
            e_theta = theta_goal - pose[2]
            # adjusting the speed and angle according to the error
            velocity_msg.linear.x = max(0.5 - (0.08 * abs(e_theta)), 0.05)
            velocity_msg.angular.z = 3 * e_theta
            pub.publish(velocity_msg)

        # Tackling the concave obstacle using Bug Algorithm 2
        goal = [12.5, 0]
        # Moving the bot till the goal is reached i.e. distance between bot and goal != 0
        while math.sqrt((pose[0] - goal[0])**2 + (pose[1] - goal[1])**2) > 0.1:
            # Going straight towards the goal until the obstacle or goal comes
            while (regions['front'] > 1) and (math.sqrt((pose[0] - goal[0])**2 + (pose[1] - goal[1])**2) > 0.1):
                waypoint_goal = Waypoints(pose[0])
                theta_goal = math.atan2(-pose[1], 0.1)
                e_theta = theta_goal - pose[2]
                velocity_msg.linear.x = max(0.5 - (0.08 * abs(e_theta)), 0.05)
                velocity_msg.angular.z = 4 * e_theta
                pub.publish(velocity_msg)
            # If the obstacle appears bot follows a clockwise path along the obstacle
            if regions['front'] < 1:
                encountered_wall_at = [pose[0], pose[1]]
                encountered_wall_at_distance = math.sqrt(
                    (encountered_wall_at[0] - goal[0])**2 + (encountered_wall_at[1] - goal[1])**2)
# A: The bot is near the line y = 0
# B: The bot is near the place where it first encountered the wall
# C: distance between the bot and goal increases
# Bot keeps following the wall = !(Bot stops following the wall) = !(A and !B and !C) = !A or B or C
                while not(-0.4 < pose[1] < 0.4) or (math.sqrt((encountered_wall_at[0] - pose[0])**2 + (encountered_wall_at[1] - pose[1])**2) < 1.5) or (math.sqrt((goal[0] - pose[0])**2 + (goal[1] - pose[1])**2) > encountered_wall_at_distance):
                    # if the front side of the bot is too close to the wall
                    if regions['front'] < 1:
                        velocity_msg.linear.x = 0.1
                        velocity_msg.angular.z = 5 * (1 - regions['front'])
                    # if right side of the bot is too close to the wall
                    elif regions['fright'] < 0.3:
                        velocity_msg.linear.x = 0.2
                        velocity_msg.angular.z = 1
                    # if right side of the bot is too far from the wall
                    elif regions['fright'] > 0.8:
                        velocity_msg.linear.x = 0.2
                        velocity_msg.angular.z = -1.5
                    # if bot is at a safe distance from the wall
                    else:
                        velocity_msg.linear.x = 0.5
                        velocity_msg.angular.z = 0
                    pub.publish(velocity_msg)
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
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
    range_max = 25.0
    regions = {
        'bright': min(min(msg.ranges[0:144]), range_max),
        'fright': min(min(msg.ranges[144:288]), range_max),
        'front': min(min(msg.ranges[288:432]), range_max),
        'fleft': min(min(msg.ranges[432:576]), range_max),
        'bleft': min(min(msg.ranges[576:720]), range_max),
    }


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
