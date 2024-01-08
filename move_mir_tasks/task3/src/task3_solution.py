#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import math
import copy
import time

class Task3Node():
    def __init__(self, topic: str ="odometry"):
        rospy.init_node("Task3", anonymous=True)
        self.rate = rospy.Rate(1)
        self.initial_position = 0
        self.indicator = 0
        self.current_position = 0
        self.pos_diff= -1
        self.speed = Odometry()
        if topic.lower() == "odometry":
            self.publisher = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
            self.subscriber = rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.callback)

    def callback(self, msg):
        rospy.loginfo("Current position: x = {0}, y = {1} and z = {2} \n".format(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
        if self.indicator == 0:
            self.initial_position = float(msg.pose.pose.position.x)
            self.indicator+=1
           
        self.current_position = float(msg.pose.pose.position.x)
    
    def run(self):
        while not rospy.is_shutdown():
            while self.current_position - self.initial_position < 1:
                self.pos_diff = self.current_position - self.initial_position
                self.speed.twist.twist.linear.x = 0.2
                self.publisher.publish(self.speed.twist.twist)
                rospy.loginfo ("The robot moves. Position before {0}, now {1}".format(self.initial_position, self.current_position))
            self.speed.twist.twist.linear.x = 0.0
            self.indicator = 0
            self.publisher.publish(self.speed.twist.twist)
            self.rate.sleep()
            rospy.loginfo ("The robot stooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooops position difference {0}".format(self.pos_diff))
            
            
            
def main(args=None):
    #rospy.init(args=args)
    try:
        test_node = Task3Node()
        test_node.run()
    except rospy.ROSInterruptException:
        pass
    #rospy.spin()

if __name__ == "__main__":
    

        main()
        