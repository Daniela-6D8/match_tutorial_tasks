import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import math

def move ( velocity, distance, is_forward):
    vel_msg = Twist()
    rospy.init_node ('move_control_M100', anonymous=True)
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist,queue_size=10 )

    global x
    global y

    x0 = x
    y0= yield

    if (is_forward):
        vel_msg.linear.x = abs (velocity)
    else :
       vel_msg.linear.x = -abs(velocity)
    
    distance_moved =0.0
    rate = rospy.Rate(10)

    while True:
        rospy.loginfo(" robot move forward")
        pub.publish(vel_msg)
        rate.sleep()
        distance_moved = abs (  math.sqrt(((x-x0)**2)+ (y-y0)**2) )


    if __name__ == '__main__':

        while not rospy.is_shutdown:
           move(0,5)