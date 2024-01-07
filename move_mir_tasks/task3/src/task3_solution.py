#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import copy
import time

class Task3Node():
    def __init__(self, topic: str ="odometry"):
        rospy.init_node("Task3", anonymous=True)
        self.rate = rospy.Rate(5)
        self.state = 0
        self.current_pose = Odometry()
        self.final_pose = 0
        self.speed = Twist()
        if topic.lower() == "odometry":
            self.publisher = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
            self.subscriber = rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.callback)

    def callback(self, msg):
        rospy.loginfo("Current position: x = {0}, y = {1} and z = {2} \n".format(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
    
    def run(self):
        while not rospy.is_shutdown():
             #rospy.loginfo("Current position: x = {0}, y = {1} and z = {2} \n".format(self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y, self.current_pose.pose.pose.position.z))
    
            if self.state == 0 :
                self.final_pose = self.current_pose.pose.pose.position.x + 1
                print("Ici",self.final_pose , self.current_pose.pose.pose.position.x, flush=True)
                self.state+=1
            elif self.state == 1:
                while self.final_pose > self.current_pose.pose.pose.position.x :
                    print("Code herev 11111111111111111", self.final_pose - self.current_pose.pose.pose.position.x, flush=True)
                    self.speed.linear.x = 0.2
                    self.publisher.publish(self.speed)
                    print ("The robot moves")
                print("Code herev 2222222222222222", flush=True)
                self.speed.linear.x = 0
                self.publisher.publish(self.speed)
                print ("The robot stops") 
                time.sleep(1)
                
                self.state = 0
            
            else :
                pass    
            
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
        