#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

p_current = Pose()
p_end = Pose()
twist = Twist()

# Orientation saved as Euler angles
yaw_end = 0.0
yaw_current = 0.0

# States:
# 0: Start up, set the first end position
# 1: Move to the first end position
# 2: Set the new end pose with position and orientation
# 3: Rotate to end orientation
# 4: Move to end position
# 5: Finish
state = 0

p = None

def run():
    global state, p_current, p_end, twist, yaw_end, yaw_current

    # Debug Message
    rospy.loginfo("running state: %i", state)

    # State machine
    if state == 0:
        p_end = p_current
        p_end.position.x += 1
        state += 1

    elif state == 1:
        if p_current.position.x < p_end.position.x:
            twist.linear.x = 0.3
        else:
            twist.linear.x = 0
            state += 1

    elif state == 2:
        p_end = p_current
        p_end.position.x -= 1
        yaw_end = math.pi
        q_end = quaternion_from_euler(0, 0, yaw_end)
        state += 1

    elif state == 3:
        q_current = (
            p_current.orientation.x,
            p_current.orientation.y,
            p_current.orientation.z,
            p_current.orientation.w
        )
        (roll, pitch, yaw_current) = euler_from_quaternion(q_current)

        if abs(yaw_end - yaw_current) > 0.05:
            twist.angular.z = 0.5
        else:
            twist.angular.z = 0
            state += 1

    elif state == 4:
        if p_current.position.x > p_end.position.x:
            twist.linear.x = 0.3
        else:
            twist.linear.x = 0
            state += 1

    elif state == 5:
        rospy.loginfo("Finished!")

    p.publish(twist)

def callback(msg):
    global p_current
    p_current = msg.pose.pose
    run()                                          ## Aufpassen!!!

def main():
    global p

    rospy.loginfo("started")
    rospy.init_node("solution_node2")
    rate = rospy.Rate(5)
    p = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
    s = rospy.Subscriber("/mobile_base_controller/odom", Odometry, callback)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass









# class Robot_move ():

#     def __init__ (self):
#         rospy.init_node ('move_control_M100', anonymous=True)
#         self.pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist,queue_size=10 )
#         self.speed = Twist()
#         self.pose = Pose()
#         self.x = self.pose.position.x 
#         self.y= self.pose.position.y
#         self.distance_moved = 0.0 

#         self.rate =rospy.Rate(10)
#         self.sub = rospy.Subscriber ('/mir_pose_simple', Pose , self.callback_fontion  )
        
#         #self.x0=self.x
#         #self.y0=self.y

#     def pub_vel(self,distance): 

#         self.speed.linear.x = 0.2
#         self.pub.publish (self.speed)
#         self.rate.sleep() 
#         distance_moved = abs ( math.sqrt(self.x-self.x0)**2+(self.y-self.y0)**2)
#         if distance <  distance_moved:
#              rospy.loginfo (distance_moved)
#              self.speed.linear.x = 0

#     def stop_robot(self):
#         self.speed.linear.x = 0
#         self.pub.publish (self.speed)
#        #self.speed.linear.x = vel_x   
       

#     def callback_fontion(self,msg):
#         rospy.sleep(0.5)
#         if self.speed.linear.x < 0.02: 

#             rospy.loginfo (" the robot doesn't move")    
            
#         else:
#             rospy.loginfo (" new position recieved")
#             rospy.loginfo (msg.position)
           
        
    
#     def position_orientation():
#         return

# if __name__ == '__main__':

#     robot = Robot_move() 

#     while not rospy.is_shutdown():
           
#         robot.pub_vel(1)
#         #rospy.sleep(10)
#         #robot.stop_robot()
#         #rospy.sleep(0.1)

