#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


##### test of Quaternion   #####

# if __name__ == '__main__':
    
#       # RPY to convert: 90deg, 0, -90deg
#       q = quaternion_from_euler(90, 0, -90)
#       yaw_end = math.pi

#       print (yaw_end)
   
#       print ("The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3]))

### END  ###


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
    move ()






    

    


# p_current = Pose()
# p_end = Pose()
# twist = Twist()

# # Orientation saved as Euler angles
# yaw_end = 0.0
# yaw_current = 0.0

# # States:
# # 0: Start up, set the first end position
# # 1: Move to the first end position
# # 2: Set the new end current_pose with position and orientation
# # 3: Rotate to end orientation
# # 4: Move to end position
# # 5: Finish
# state = 0

# p = None

# def run():
#     global state, p_current, p_end, twist, yaw_end, yaw_current

#     # Debug Message
#     rospy.loginfo("running state: %i",current_pose state)

#     # State machine
#     if state == 0:
#         p_end = p_current
#         p_end.position.x += 1
#         state += 1

#     elif state == 1:
#         if p_current.position.x < p_end.position.x:
#             twist.linear.x = 0.3
#         else:
#             twist.linear.x = 0
#             state += 1

#     elif state == 2:
#         p_end = p_current
#         p_end.position.x -= 1
#         yaw_end = math.pi
#         q_end = quaternion_from_euler(0, 0, yaw_end)
#         state += 1

#     elif state == 3:
#         q_current = (
#             p_current.orientation.x,
#             p_current.orientation.y,
#             p_current.orientation.z,
#             p_current.orientation.w
#         )
#         (roll, pitch, yaw_current) = euler_from_quaternion(q_current)

#         if abs(yaw_end - yaw_current) > 0.05:
#             twist.angular.z = 0.5
#         else:
#             twist.angular.z = 0
#             state += 1

#     elif state == 4:
#         if p_current.position.x > p_end.position.x:
#             twist.linear.x = 0.3
#         else:
#             twist.linear.x = 0
#             state += 1

#     elif state == 5:
#         rospy.loginfo("Finished!")

#     p.publish(twist)

# def callback(msg):
#     global p_current
#     p_current = msg.pose.pose
#     run()                                          ## Aufpassen!!!

def main():
    global pub

    rospy.loginfo("started")
    rospy.init_node("solution_node2")
    rate = rospy.Rate(5)
    pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
    s = rospy.Subscriber("/mobile_base_controller/odom", Odometry, callback)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":

    try:
          main()

    except rospy.ROSInterruptException:
        pass

