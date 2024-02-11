# multi-robot_exploration_mapping

## Project Description

A multirobot frontier explorer has been developed to autonomously explore an environment and generate a single merged global map using four different TurtleBot3 robots in the ROS1 Noetic environment.


## Environment Setup

The node configurations for the turtlebot3_navigation and turtlebot3_gazebo packages from the [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) and [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) repositories should be set up for 4 tb3 robots. The necessary configurations for multirobot have been made in the micromouse_maze environment for testing.

The required node connections following the adjustments are provided in the following tf tree.


### TF Tree Graph

![tf_tree](https://github.com/fbasatemur/multi-robot_exploration_mapping/blob/main/docs/tf_tree_last.png?ref_type=heads)

### The initialization of multi-robot_exploration_mapping:

```bash
$ sudo apt-get install ros-noetic-multirobot-map-merge
$ mkdir -p  ~/multi_tb3_fe/src
$ cd ~/multi_tb3_fe/src
$ git clone https://github.com/fbasatemur/multi-robot_exploration_mapping.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ mv multi-robot_exploration_mapping*/micromouse_maze .
$ mv multi-robot_exploration_mapping*/multirobot_exploration_mapping .
$ rm -r multi-robot_exploration_mapping*
$ cd ~/multi_tb3_fe/
$ rosdep install --from-paths src --ignore-src --rosdistro noetic -y
$ catkin_make
$ source ~/.bashrc
```

### Starting the simulation environment:

Four Turtlebot3 robots will be operated, utilizing the burger model due to the absence of the camera-depth sensor. Therefore, the TURTLEBOT3_MODEL variable in the ./bashrc file should be assigned an appropriate value.

```bash
gedit ~/.bashrc
```
```bash
export TURTLEBOT3_MODEL=burger
```

Starting the simulation environment:

```
$ roslaunch micromouse_maze micromouse_maze3_multi.launch
$ roslaunch turtlebot3_gazebo multi_map_merge.launch
$ roslaunch turtlebot3_gazebo multi_turtlebot3_slam.launch
$ roslaunch turtlebot3_navigation multi_move_base.launch
$ roslaunch micromouse_maze multi_robot_rviz.launch
```

Starting the multi-robot frontier explorer node:

```bash
rosrun multirobot_exploration_mapping multi_tb3_fe
```

# multi_tb3_fe

For each Turtlebot3, after the map merge process, the /map frame is listened to, and the frontier explorer algorithm is executed for path planning. Path planning is performed again for each robot that either reaches the goal or gets stuck due to an obstacle.

The FE planning for each robot is carried out with the main_loop() function. Each robot becomes a client to the /tb3_*/move_base/ node for receiving movement commands and monitoring its status.



## Frontier Explorer Steps

The FE planning for robots consists of the following steps:

**Finding Border Edges and Regions:**
When the /map frame is received, the find_all_edges() function is called to locate the border edges.
If there are no border edges, the loop proceeds to the end. If border edges are found, the find_regions() function is called to determine the border regions and their centroids.

**Determining Robot Position:**
Once the map and border regions are determined, the transformation between the robot's body frame and the map frame is found using find_transform(), yielding the robot's position.

**Assignment of Target:**
If border regions exist, the closest centroid is determined with find_closest_centroid() and the robot moves towards this centroid. If there are no border regions, the closest border edge is determined, and the robot moves towards it.

**Movement and Robot State Monitoring:**
After the target is determined, a movement command is sent to the tb3_*/move_base node.
The robot's progress towards the goal is monitored via the getState() method of the MoveBaseAction package.

These steps describe the process followed by each Turtlebot3 for frontier exploration and path planning.


## Conclusion

### multi_tb3_fe outputs

Since the FE planning for robots is done using threads, there is a race condition in the terminal outputs. Therefore, each robot's output is labeled with "[tb3_*]" to accurately observe the status information of each robot. 

<img src="https://github.com/fbasatemur/multi-robot_exploration_mapping/blob/main/docs/multi_tb3_console.png?ref_type=heads" width="400" height="527"/>

### rviz results

The initial state and the progress in subsequent stages:

![mrfe1](https://github.com/fbasatemur/multi-robot_exploration_mapping/blob/main/docs/multi_tb3_fe_c1.png?ref_type=heads)

![mrfe2](https://github.com/fbasatemur/multi-robot_exploration_mapping/blob/main/docs/multi_tb3_fe_c2.png?ref_type=heads)
