#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

using namespace ros;

geometry_msgs::Pose p_current;
geometry_msgs::Pose p_end;
geometry_msgs::Twist twist;
geometry_msgs::Pose poseAMCLx, GTx;

// Orientation saved as Quaternion
geometry_msgs::Quaternion poseAMCLo, GTo;
geometry_msgs::Quaternion q_current;
tf2::Quaternion q_end;

// yaw_end will be the end orientation of the rotation
// yaw_current will be the current orientation of the robot
// junk is for not needed references in the GetEulerYPR() function
double yaw_end, yaw_current, junk;

// Quaternion from tf2 namespace to set new orientation with rad (see "ros tf2" tutorial)
tf2::Quaternion q_tf;


int state = 0;

Publisher p;

int round_num = 1;

void moveOneStep(geometry_msgs::Pose pose){

    if(abs(p_end.position.x - pose.position.x) > 0.1 ){
        twist.linear.x = 0.3;
    }
    else{
        twist.linear.x = 0;
        state++;
    }
}

void turn(geometry_msgs::Quaternion quat){

    tf2::Quaternion tf2q_current(
    quat.x,
    quat.y,
    quat.z,
    quat.w);
    tf2::Matrix3x3 matrix_current(tf2q_current);
    matrix_current.getRPY(junk, junk, yaw_current);
    if (abs(yaw_end - yaw_current) > 0.05)
    {
        twist.angular.z = 0.5;
    }
    else
    {
        twist.angular.z = 0;
        state++;
    }
}


void run(geometry_msgs::Pose pose, geometry_msgs::Quaternion quat)
{   
    // Debug Message
    ROS_INFO("running state: %i", state);

    switch (state)
    {
    case 0:
        {
            p_end = pose;
            p_end.position.x = 1;


            state++;
        }
        break;
    case 1:    
        {
            moveOneStep(pose);  
        }
        break;
    case 2:
        {   
            ROS_INFO("end: %f", p_end.position.x);
            ROS_INFO("x: %f", pose.position.x);

            yaw_end = M_PI;
            turn(quat);
        }
        break;

    case 3:
        {
            p_end = pose;
            p_end.position.x = 0;
            ROS_INFO("end: %f", p_end.position.x);
            ROS_INFO("x: %f", pose.position.x);
            state++;
        }
        break;
    case 4:
        {
            moveOneStep(pose);  
        }
        break;
    case 5:
        {
            yaw_end = 0;
            turn(quat);
        }
        break; 
    default:
        {
            state=0;
            round_num++;
        }
        break;       
    }

    p.publish(twist);
}


void OdomCallback(const nav_msgs::Odometry msg)
{
    p_current = msg.pose.pose;
    q_current = msg.pose.pose.orientation;
    if(round_num % 3 == 1){
        ROS_INFO("odom running");
        run(p_current,q_current);
    }
}



void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    poseAMCLx = msgAMCL->pose.pose;
    poseAMCLo = msgAMCL->pose.pose.orientation;   
    if(round_num % 3 == 2){
        ROS_INFO("amcl running");
        run(poseAMCLx,poseAMCLo);
    }
}

void GTCallback(const nav_msgs::Odometry msgGT)
{
    GTx = msgGT.pose.pose;
    GTo = msgGT.pose.pose.orientation;   
    if(round_num % 3 == 0){
        ROS_INFO("ground truth running");
        run(GTx,GTo);
    }
}

int main(int argc, char **argv)
{
    ROS_INFO("started");
    init(argc, argv, "solution_node3");
    NodeHandle n;
    Rate loop_rate(5);

    p = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1000);
    
    Subscriber s = n.subscribe("/mobile_base_controller/odom", 1, OdomCallback);
   
    Subscriber sub_amcl = n.subscribe("/amcl_pose", 1, poseAMCLCallback);

    Subscriber sub_gt = n.subscribe("/ground_truth", 1, GTCallback);

    spin();

    return 0;
}
