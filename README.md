# Tutorial files for working with the Match_Mobile_Robotics repository




The following procedure has to be done for each tutorial independently:
>1. Start the world/environment:
>   * Simulation a): `roslaunch match_gazebo world_{world_name}.launch`
>   * Simulation b): `roslaunch gazebo_ros {world_name}.launch`
>   * Hardware: This us usually not necessary since the real world just exits ^^.
>2. Bringup the specified hardware driver: 
>   * Simulation: `roslaunch mir_launch_sim mir.launch`
>   * Hardware :
>   ```
>   ssh {robot_name}
>   roslaunch mir_launch_hardware mir.launch
>   ```
>3. Use the tutorial file you want to explore:
>   * Simulation & Hardware: `roslaunch tutorial_mir {file_name}.launch`

# The different tutorials
## Move the platform with a ps4 controller
The mobile plattform can be easily handled by using the playstation 4 controller. This controller controls the velocities of the plattform based on the users input (R2/L2: translational velocity, Right stick: Rotational velocity).

Try it:
```
roslaunch tutorial_mir mir_ps4_drive.launch
```
Understand what is happening by calling:
```
rosrun rqt_graph rqt_graph 
```
You receive a node graph that contains a part like:

<img src="figures/ps4_drive.png" width="50%">

This two nodes and topics are directly related to the calls within the mir_ps4_drive.launch (See commentary within the file!).


## Move base action server
The move base action server is used for moving the plattform to a specific position in space. This server is launched defaultly with the hardware.

Try it:

```
roslaunch tutorial_mir mir_move_base_drive.launch
```
You receive a rviz visualisation simmilar to:

<img src="figures/move_base.png" width="50%">

Since this action server itself and a the map are launched defaultly with the hardware, the launchfile only contains the properly configured visualisation for interacting with the server. For interacting with the move base server you may click on the 2D Nav Goal button an specify a pose within the map. As this speciifcation is done the move base node determines a global path to this pose and local path planning is respecting dynamically occuring obstacles.

## Manage multi robot applications
Within multi robot applications collisions of resource names have to be avoided. Such resources are e.g. node names, topic names, frame names. Such collisions are avoided by launchfile arguments within `mir_launch_sim` and `mir_launch_hardware`. To understand these arguments the files mir_spawn_named.launch and mir_named.launch are given.

Try it by substituting step 2. from the initial setup routine (cmp. top):

>2. Bringup the specified hardware driver: 
>   * Simulation: `roslaunch mir_launch_sim mir.launch`
>   * Hardware :
>   ```
>   ssh {robot_name}
>   roslaunch mir_launch_hardware mir.launch
>   ```

Understand what is happening to topics and nodes by calling:
```
rosrun rqt_graph rqt_graph 
```
You receive a node graph simmilar to

<img src="figures/multi_mir_named.png" width="50%">

wich was 

<img src="figures/multi_mir.png" width="50%">



You can see that each node and topic got his own namespace, given by the robot_name argument within the launchfiles (See commentary within the launchfile).

The same behaviour can be seen for the tf frames as you call:

```
rosrun  rqt_tf_tree rqt_tf_tree 
```
This leads to a tf tree simmilar to

<img src="figures/multi_mir_named_tf.png">

wich was 

<img src="figures/multi_mir_tf.png">


