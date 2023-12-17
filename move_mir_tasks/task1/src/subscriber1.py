#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import time

class RobotController:
    def __init__(self):
        rospy.init_node('move_and_stop_robot', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.odom_callback)

        self.move_cmd = Twist()
        self.distance_moved = 0.0

        # Move the robot for 2 seconds
        self.move_robot(duration=2)

    def move_robot(self, duration):
        self.move_cmd.linear.x = 0.8  # Set linear velocity for movement
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.cmd_vel_pub.publish(self.move_cmd)
            rospy.sleep(0.01)  # Adjust this value based on your control loop rate

        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        # Calculate the distance moved
        rospy.loginfo("Robot moved for {} seconds.".format(duration))
        rospy.loginfo("Distance moved: {} meters".format(self.distance_moved))

    def odom_callback(self, odom_msg):
        # Extract orientation from odometry message
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        # Update distance moved based on linear velocity
        linear_velocity = odom_msg.twist.twist.linear.x
        elapsed_time = (rospy.Time.now() - odom_msg.header.stamp).to_sec()

        distance_increment = linear_velocity * elapsed_time
        self.distance_moved += distance_increment

if __name__ == "__main__":
    controller = RobotController()
    rospy.spin()
