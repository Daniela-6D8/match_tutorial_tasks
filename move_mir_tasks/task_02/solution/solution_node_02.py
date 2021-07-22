#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from tf import transformations
import math

class solution_node():

    def __init__(self):
        rospy.init_node("solution_node")
        self.current_robot_pose = Pose()
        self.starting_point = Pose()
        self.twist = Twist()
        self.state = 0
        self.first_call = True
        self.cmd_vel_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size= 1)
        rospy.Subscriber("/mobile_base_controller/odom",Odometry, self.odometry_cb)
        rospy.spin()
        
    def run(self):
        rospy.loginfo_throttle(1,"running")

        if self.first_call:
            self.starting_point = self.current_robot_pose
            self.first_call = False 
        elif self.state == 0:
            if self.current_robot_pose.position.x < self.starting_point.position.x+1:
                self.twist.linear.x = 0.1



            else: 
                self.twist.linear.x = 0.0
                self.state = 1

        elif self.state == 1:
            rot = transformations.quaternion_about_axis(math.pi, (0,0,1))
            q = [self.starting_point.orientation.x,self.starting_point.orientation.y,self.starting_point.orientation.z,self.starting_point.orientation.w]
            q_target = transformations.quaternion_multiply(q,rot)

            target_orientation = transformations.euler_from_quaternion(q_target)
            current_orientation = transformations.euler_from_quaternion([self.current_robot_pose.orientation.x,self.current_robot_pose.orientation.y,self.current_robot_pose.orientation.z,self.current_robot_pose.orientation.w])

            print(abs(target_orientation[2] - current_orientation[2]))
            if abs(target_orientation[2] - current_orientation[2]) > 0.05:
                self.twist.angular.z = 0.2
            else: 
                self.twist.angular.z = 0.0
                self.state = 2

        elif self.state == 2:
            if self.current_robot_pose.position.x > self.starting_point.position.x:
                self.twist.linear.x = 0.1
            else: 
                self.twist.linear.x = 0.0

        self.cmd_vel_pub.publish(self.twist)


    def odometry_cb(self,data):     # type: (Odometry) -> Pose
        self.current_robot_pose = data.pose.pose
        self.run()
        
if __name__=="__main__":
    Solution_node = solution_node()
    Solution_node.run()
    