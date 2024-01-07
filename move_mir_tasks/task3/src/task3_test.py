#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


state = 0
current_pose = Pose()
end_pose = Pose ()
speed = Twist()
current_q = 0
end_q = 0
pub = None



def move ():
   
    global pub, current_pose , end_pose , speed , end_yaw, current_yaw , state , end_q , current_q

     # Debug Message
    rospy.loginfo ("running state: %i",state)

    if state == 0 :
        current_pose = end_pose
        end_pose.position.x+=1
        state+=1

    elif state == 1:

        if end_pose.position.x > current_pose.position.x :
            speed.linear.x = 0.2
            print ("the robot moves")
        else :
            speed.linear.x = 0
            print ("the robot is stoped") 
            state+=1
            
    elif state == 2 :
        current_pose = end_pose
        end_pose.position.x -= 1
        end_yaw = math.radians(180)
        end_q = quaternion_from_euler(0, 0, end_yaw)
        state+=1

    elif state == 3 :
        difference_angle = math.radians (2)
        current_q = (
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w )
        (roll, pitch, current_yaw ) = euler_from_quaternion(current_q)

        if abs(end_yaw - current_yaw) > difference_angle :
            speed.angular.z = 0.5 
            print (" the robot rotates")
        else :
            speed.angular.z = 0.0
            print (" the robot doesn't rotate anymore")
            state += 1

    elif state == 4 :

        if current_pose.position.x > end_pose.position.x :
            speed.linear.x = 0.2
            print ("the robot moves back")
        else :
            speed.linear.x = 0
            print ("the robot is stoped") 
            state+=1

    elif state == 5:
        print (" state machine completed")

    pub.publish (speed)
        

def callback (msg) :
    global current_pose 
    current_pose = msg.pose.pose
    move()                                      ## Aufpassen!!!

                                   

def main():
    global pub , pub_2  
    rospy.loginfo("started")
    rospy.init_node("solution_node2")
    rate = rospy.Rate(5)
    pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)

    pub_2 = rospy.Publisher("/ground_truth", Odometry, queue_size=10)
    s = rospy.Subscriber("/mobile_base_controller/odom", Odometry, callback)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":

    try:
          main()

    except rospy.ROSInterruptException:
        pass

