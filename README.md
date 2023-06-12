# Tutorial tasks to learn about the Match_Mobile_Robotics Repository 

# Task 1

Create a launch file that spawns a single MiR100 robot platform in the "empty_world" map. Than write a node that moves the robot 1 m in x direction with C++ or Python.
Hints

Create a publisher and subscriber in one node to receive/send the messages from/to the robot. Use the WritingPublisherSubscriber ROS examples for C++ or Python
To find out which one to use, use the rostopic list command
Use rostopic type "topic" to see the data type of a topic
Set up your CMakeLists.txt correctly
If you want to test your programm you have to start your .launch file first

# Task 2

Now let the robot rotate 180 degrees and move back to the starting point in C++ or Python

### Hints
- Use the following activity diagram to code a state machine

![Activity diagram](https://i.ibb.co/cDJ16p1/Diagram-2022-12-02-13-56-49.png "Activity diagram")
- For the orientation you need to work with quaternions. Use the [TF2 tutorial](http://wiki.ros.org/tf2/Tutorials/Quaternions "TF2 tutorial") for that 

# Task 3 (work in progress)

Write a loop for each of the following pose topics that completes the tasks 01 and 02 an infinite number of times:

Gazebo ground_truth
AMCL
Odometry


