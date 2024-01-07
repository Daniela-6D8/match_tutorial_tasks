#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Position of odometry, AMCL, and Ground Truth
p_current = Pose()
poseAMCLx = Pose()
GTx = Pose()

# Orientation of odometry, AMCL, and Ground Truth
q_current = p_current.orientation
poseAMCLo = poseAMCLx.orientation
GTo = GTx.orientation

# Variable for publishing twist
twist = Twist()

# Pose and orientation of the end goal
p_end = Pose()
q_end = quaternion_from_euler(0, 0, 0)  # Initialize with no rotation

# Yaw_end will be the end orientation of the rotation
# Yaw_current will be the current orientation of the robot
# Junk is for not needed references in the euler_from_quaternion function
yaw_end, yaw_current, junk = euler_from_quaternion([q_end.x, q_end.y, q_end.z, q_end.w])

# State variable
state = 0

"""
States:
0: Start up, set the first end position
1: Move to the first end position
2: Turn 180 degrees
3: Set the new end pose with position and orientation
4: Move to end position
5: Turn 180 degrees

default: set the state = 0
"""

# Publisher for twist messages
p = None

round_num = 1

# round_num % 3:
# 1: Run using Odometry poses
# 2: Run using AMCL poses
# 0: Run using Ground Truth poses


def move(pose):
    global twist
    # Move the robot towards the end goal
    if abs(p_end.position.x - pose.position.x) > 0.1:
        twist.linear.x = 0.3
    else:
        twist.linear.x = 0
        global state
        state += 1


def turn(quat):
    global twist, yaw_end, yaw_current
    # Turn the robot's yaw towards yaw end
    _, _, yaw_current = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    if abs(yaw_end - yaw_current) > 0.05:
        twist.angular.z = 0.5
    else:
        twist.angular.z = 0
        global state
        state += 1


def run(pose, quat):
    # Debug Message
    rospy.loginfo("running state: %i", state)

    global p_end, state, round_num, twist
    global yaw_end

    if state == 0:
        p_end = pose
        p_end.position.x = 1
        state += 1

    elif state == 1:
        move(pose)

    elif state == 2:
        rospy.loginfo("end: %f", p_end.position.x)
        rospy.loginfo("x: %f", pose.position.x)

        yaw_end = 3.14159
        turn(quat)

    elif state == 3:
        p_end = pose
        p_end.position.x = 0
        rospy.loginfo("end: %f", p_end.position.x)
        rospy.loginfo("x: %f", pose.position.x)
        state += 1

    elif state == 4:
        move(pose)

    elif state == 5:
        yaw_end = 0
        turn(quat)

    else:
        state = 0
        round_num += 1

    p.publish(twist)


def odom_callback(msg):
    # Callback function for Odometry data
    global p_current, q_current, round_num
    p_current = msg.pose.pose
    q_current = msg.pose.pose.orientation
    if round_num % 3 == 1:
        rospy.loginfo("odom running")
        run(p_current, q_current)


def pose_amcl_callback(msg):
    # Callback function for AMCL data
    global poseAMCLx, poseAMCLo, round_num
    poseAMCLx = msg.pose.pose
    poseAMCLo = msg.pose.pose.orientation
    if round_num % 3 == 2:
        rospy.loginfo("amcl running")
        run(poseAMCLx, poseAMCLo)


def gt_callback(msg):
    # Callback function for Ground Truth data
    global GTx, GTo, round_num
    GTx = msg.pose.pose
    GTo = msg.pose.pose.orientation
    if round_num % 3 == 0:
        rospy.loginfo("ground truth running")
        run(GTx, GTo)


def main():
    global p, round_num

    rospy.init_node("solution_node3")
    rospy.loginfo("started")
    rate = rospy.Rate(5)

    # Publishing Twist
    p = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=1000)

    # Subscribing to Odometry data
    rospy.Subscriber("/mobile_base_controller/odom", Odometry, odom_callback)

    # Subscribing to AMCL data
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_amcl_callback)

    # Subscribing to Ground Truth data
    rospy.Subscriber("/ground_truth", Odometry, gt_callback)

    # All user callbacks will be called from within the rospy.spin() call.
    # rospy.spin() will not return until the node has been shut down,
    # either through a call to rospy.shutdown() or a Ctrl-C.
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
