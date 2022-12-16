#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class solution_node():

    def __init__(self):
        # Set up the variables, publisher and subscriber
        rospy.init_node("solution_node")
        
        # Pose is a message that contains the position (Point) and the orientation (Quaternion)
        self.current_robot_pose = Pose()
        self.target_robot_pose = Pose()
        self.twist = Twist()
        self.first_call = True

        # To find out which one to use, enter "rostopic list" in the command line
        # Use "rostopic type <topic>" to see the data type of a topic.
        self.cmd_vel_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size= 1)
        rospy.Subscriber("/mobile_base_controller/odom",Odometry, self.odometry_cb)
        
        # Important: Don't use spinOnce() because you want a continuous Subscriber
        rospy.spin()
        
    def run(self):
        rospy.loginfo_throttle(1,"running")
        # If the current Position was received for the first time
        # the end position is set
        if self.first_call:
            self.target_robot_pose = self.target_robot_pose
            self.target_robot_pose.position.x += 1
            self.first_call = False 
        else:
            if self.current_robot_pose.position.x < self.target_robot_pose.position.x:
                # Set x speed to 0.5
                self.twist.linear.x = 0.1
            else: 
                # Stop the robot
                self.twist.linear.x = 0.0
            # Publish twist message to the cmd_vel topic of the robot
            self.cmd_vel_pub.publish(self.twist)


    def odometry_cb(self,data):     # type: (Odometry) -> Pose
        # Write the current position of the robot from the message to p_current
        self.current_robot_pose = data.pose.pose
        self.run()
        
if __name__=="__main__":
    Solution_node = solution_node()
    Solution_node.run()
    
