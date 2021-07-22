#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class solution_node():

    def __init__(self):
        rospy.init_node("solution_node")
        self.current_robot_pose = Pose()
        self.target_robot_pose = Pose()
        self.twist = Twist()
        self.first_call = True
        self.cmd_vel_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size= 1)
        rospy.Subscriber("/mobile_base_controller/odom",Odometry, self.odometry_cb)
        rospy.spin()
        
    def run(self):
        rospy.loginfo_throttle(1,"running")
        if self.first_call:
            self.target_robot_pose = self.target_robot_pose
            self.target_robot_pose.position.x += 1
            self.first_call = False 
        else:
            if self.current_robot_pose.position.x < self.target_robot_pose.position.x:
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
    