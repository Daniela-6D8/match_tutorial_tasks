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
        self.speed.linear.x = 0.2
        self.rate =rospy.Rate(10)
        self.sub = rospy.Subscriber ('/mir_pose_simple', Pose , self.callback_fontion  )
        self.distance = 0

    def pub_vel(self):
        self.distance += 0.2
        self.pub.publish (self.speed)
        self.rate.sleep()  
    
    def stop_robot(self):
        self.speed.linear.x = 0
        self.pub.publish (self.speed)
       #self.speed.linear.x = vel_x   
       

    def callback_fontion(self,msg):
        #self.position = Pose()
        rospy.loginfo ("  new position recieved")
        rospy.loginfo (msg.position)
        time.sleep(0.5)
    

if __name__ == '__main__':

    robot = Robot_move()
    while not rospy.is_shutdown():
        #pos = Pose()
        #rospy.loginfo(pos.position)
        if not robot.distance > 10:
            robot.pub_vel()
        #rospy.sleep(10)
        #robot.stop_robot()


""" 
        try:
            robot = Robot_move()    
            robot.pub_vel()
            rospy.sleep(10)
            #robot.stop_robot()
        except rospy.ROSInterruptException:
            pass
        """
       
       

        

""" def Move ():

    rospy.init_node ('move_control_M100', anonymous=True)
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10 )
    rate =rospy.Rate(10)
    speed = Twist()
    speed.linear.x = 0.4
    pub.publish (speed)
    rate.sleep()

while not rospy.is_shutdown():
    Move()
    """
    



