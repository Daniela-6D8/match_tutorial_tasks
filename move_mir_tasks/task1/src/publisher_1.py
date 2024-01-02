#!/usr/bin/env python3                             
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import time



class Robot_move:

    def __init__ (self):
        rospy.init_node ('move_control_M100', anonymous=True)
        self.pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist,queue_size=10 )
        self.pub_2 = rospy.Publisher('/mir_pose_simple', Pose, queue_size=10 )
        self.speed = Twist()
        self.position= Pose()
        self.speed.linear.x = 0.5
        self.rate =rospy.Rate(10)
        self.sub = rospy.Subscriber ('/mir_pose_simple', Pose , self.callback_fontion  )
        #self.sub = rospy.Subscriber ('/mobile_base_controller/cmd_vel', Twist, self.callback_fontion  )

        #self.sub = rospy.Subscriber ('/mobile_base_controller/odom', Odometry , self.callback_fontion  )
        self.distance = self.position.position.x
        self.current_position = 0


    def pub_vel(self):
        #self.distance += 0.2
        self.pub.publish (self.speed)
        self.pub_2.publish (self.position)
        self.rate.sleep()  
    
    def stop_robot(self):
        self.speed.linear.x = 0
        self.pub.publish (self.speed)
        #self.pub.publish (self.position)
       #self.speed.linear.x = vel_x   
       

    def callback_fontion(self,msg):
        #position = Pose()
        rospy.loginfo ("  new position recieved")
        rospy.loginfo (msg.position)
        self.current_position = msg.position.x
        rospy.loginfo ("  new position ")
        rospy.loginfo (self.current_position)
        rospy.loginfo ( self.distance)
        time.sleep(0.5)
    

if __name__ == '__main__':

    robot = Robot_move()
    while not rospy.is_shutdown():
        robot.pub_vel()
        #rospy.spin()

        if (robot.current_position - robot.distance) >0.1 :
            print("stopppppppppp!!!!!!!!!")
            #print("stopppppppppp!!!!!!!!!",flush=True)
            robot.stop_robot()


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
    



