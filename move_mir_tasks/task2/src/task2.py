#!/usr/bin/env python3                             
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import time



class Robot_move ():

    def __init__ (self):
        rospy.init_node ('move_control_M100', anonymous=True)
        self.pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist,queue_size=10 )
        self.speed = Twist()
        self.pose = Pose()
        self.pose.position.x = 0
        self.rate =rospy.Rate(10)
        self.sub = rospy.Subscriber ('/mir_pose_simple', Pose , self.callback_fontion  )

    def pub_vel(self): 
       
         
       if  self.pose.position.x < 1.0 : 

        self.speed.linear.x = 0.2
        time.sleep(5)
        self.speed.linear.x = 0.0
       else :
           self.speed.linear.x = 0.0
       self.pub.publish (self.speed)
       self.rate.sleep()  
            
    
       

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

    while not rospy.is_shutdown():
        robot = Robot_move()    
        robot.pub_vel()
        #rospy.sleep(10)
        #robot.stop_robot()
        #rospy.sleep(0.1)

