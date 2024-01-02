#!/usr/bin/env python3                             
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import time
import math



class Robot_move ():

    def __init__ (self):
        rospy.init_node ('move_control_M100', anonymous=True)
        self.pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist,queue_size=10 )
        self.speed = Twist()
        self.pose = Pose()
        """ self.x = self.pose.position.x 
        self.y= self.pose.position.y
        self.distance_moved= 0.0 """

        self.rate =rospy.Rate(10)
        self.sub = rospy.Subscriber ('/mir_pose_simple', Pose , self.callback_fontion  )
        
        #self.x0=self.x
        #self.y0=self.y

    def pub_vel(self,distance): 

        self.speed.linear.x = 0.2
        self.pub.publish (self.speed)
        self.rate.sleep() 
        # distance_moved = abs ( math.sqrt(self.x-self.x0)**2+(self.y-self.y0)**2)
        # if distance <  distance_moved:
        #     rospy.loginfo (distance_moved)
        #     self.speed.linear.x = 0

    def stop_robot(self):
        self.speed.linear.x = 0
        self.pub.publish (self.speed)
       #self.speed.linear.x = vel_x   
       

    def callback_fontion(self,msg):
        rospy.sleep(0.5)
        if self.speed.linear.x < 0.02: 

            rospy.loginfo (" the robot doesn't move")    
            
        else:
            rospy.loginfo (" new position recieved")
            rospy.loginfo (msg.position)
           
        
    
    def position_orientation():
        return

if __name__ == '__main__':

    robot = Robot_move() 

    while not rospy.is_shutdown():
           
        robot.pub_vel(1)
        #rospy.sleep(10)
        #robot.stop_robot()
        #rospy.sleep(0.1)

