# Trajectory-Planning-and-Autonomous-Navigation-ROS
The repository contains the ROS code for a trajectory planner using the A-star algorithm, localization via AMCL and obstacle detection with LiDAR. It was developed for the “Robo do DC" project at UFSCar, during the Autonomous Mobile Robots discipline in 2023.


# DC ROBOT - UFSCar

This package contains a toolkit designed for the DC Autonomous Mobile Robot. Developed as part of the Department of Computer - Autonomous Mobile Robot (DCAMR) Project, this toolkit provides essential functionalities for the DC robot's autonomous capabilities.

    	
# How to use


### Required dependencies

```bash
sudo apt-get install ros-noetic-amcl ros-noetic-costmap-converter ros-noetic-depthimage-to-laserscan ros-noetic-dynamic-reconfigure ros-noetic-ddynamic-reconfigure ros-noetic-ddynamic-reconfigure-dbgsym ros-noetic-ddynamic-reconfigure-python ros-noetic-geometry2 ros-noetic-hector-slam ros-noetic-hector-gazebo-plugins ros-noetic-move-base ros-noetic-move-base-flex ros-noetic-navigation ros-noetic-openslam-gmapping ros-noetic-rplidar-ros ros-noetic-slam-gmapping ros-noetic-spatio-temporal-voxel-layer ros-noetic-teb-local-planner ros-noetic-teleop-twist-keyboard ros-noetic-teleop-twist-joy ros-noetic-urg-node ros-noetic-rtabmap ros-noetic-rtabmap-ros ros-noetic-octomap ros-noetic-octomap-ros ros-noetic-octomap-rviz-plugins ros-noetic-octomap-server ros-noetic-octovis ros-noetic-imu-filter-madgwick ros-noetic-robot-localization ros-noetic-robot-pose-ekf ros-noetic-pointcloud-to-laserscan ros-noetic-rosbridge-server ros-noetic-map-server ros-noetic-realsense2-camera ros-noetic-realsense2-description ros-noetic-cmake-modules ros-noetic-velodyne-gazebo-plugins ros-noetic-ompl ros-noetic-navfn ros-noetic-dwa-local-planner ros-noetic-global-planner ros-noetic-costmap-2d ros-noetic-robot-self-filter ros-noetic-ros-numpy ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-grid-map-costmap-2d ros-noetic-grid-map-ros ros-noetic-grid-map-filters ros-noetic-grid-map-visualization ros-noetic-tf2-tools pcl-tools ros-noetic-roscpp ros-noetic-std-msgs ros-noetic-geometry-msgs ros-noetic-nav-msgs ros-noetic-ackermann-msgs ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-rospy ros-noetic-roslaunch ros-noetic-robot-state-publisher ros-noetic-rviz ros-noetic-joint-state-publisher-gui ros-noetic-gazebo-ros ros-noetic-orocos-kdl libopencv-dev


```

## Follow installation steps at https://github.com/vivaldini/dcrobot/tree/main, as follow:

### Creade directories
```bash
mkdir -p /home/$USER/dcrobot_ws/src
cd /home/$USER/dcrobot_ws/
```


### Initialize the Catkin workspace
```bash
catkin init
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=Release
```

### Navigate to the directory of `src` to clone the `DC Robot project`

```bash
cd /home/$USER/dcrobot_ws/src
git clone https://github.com/vivaldini/dcrobot.git
```

### Build the project
```bash
cd /home/$USER/dcrobot_ws/
catkin build
```

### Source your catkin workspace
```bash
source /home/$USER/dcrobot_ws/devel/setup.bash
```

### (optional) may you find some errors, so you can use the "Magic" of rosdep
```bash
cd /home/$USER/dcrobot_ws/src
rosdep install --from-paths . --ignore-src --os=ubuntu:focal -r -y

cd /home/$USER/dcrobot_ws/src/dcrobot
catkin build
source /home/$USER/dcrobot_ws/devel/setup.bash

```


### Next, git clone this repository
```bash
git clone https://github.com/Gabertho/Trajectory-Planning-and-Autonomous-Navigation-ROS.git
```
### Then, replace the directories envrobotz and mobile_rob_dev_sim at dcrobot_ws project for the ones in this repository.



# Adopted methodology and results

## Mapping
For the mapping step, we used gmapping (http://wiki.ros.org/gmapping), navigating the simulated environment with teleop. With this, we obtain the map.yaml and map.pgm that represent the environment.

## Localization

For localization, we use the ROS amcl package, which implements the Adaptive Monte Carlo Localizer approach. We used this algorithm, instead of using odometry data, as this can accumulate errors throughout use, leading to possible failures in trajectory planning.


## Trajectory Planning
For trajectory planning, we implemented the A* (Astar) algorithm, which calculates the optimal path between the starting and ending points, using a function composed of the cost of the movement plus a heuristic. The heuristic used was Manhattan, as it generates paths that are relatively more "distant from obstacles", such as walls, in relation to the Euclidean distance, which reduces possible collision problems.

The map, obtained in the previous step, is published by the topic /map (package map_server), and used as an Occupancy Grid in the implementation of the A* algorithm (astar.cpp and astar.h)

## Navigation 
For navigation, we use an implementation of the Pure Pursuit Path Tracking algorithm. It computes the angular velocity command that moves the robot from its current position to reach some look-ahead point in front of the robot, while the linear velocity is assumed constant. Speed ​​commands are published in the /robot/cmd_vel topic.

Here is the Pure Pursuit basics:

![image](https://github.com/Gabertho/Trajectory-Planning-and-Autonomous-Navigation-ROS/assets/59297927/ba97dad3-22a8-498d-80a7-e57c5ba30f24)


## Obstacle Avoidance
For obstacle avoidance, we implemented a "naive" approach using the LIDAR sensor. The approach consists of stopping the robot's movement on the optimal path when an obstacle is detected, and keeping the robot stationary until the obstacle is no longer detected, thus resuming its trajectory.



# Execution Flow

## astar_node.cpp

### Execution Flow and Main Components
- **Main Function**: 
  - `int main()`: Initializes the ROS node and sets up topic subscriptions and publications.
- **ROS Topics**:
  - **Subscriptions**: For receiving occupancy grid (map data), start, and target points.
  - **Publications**: For publishing the calculated optimal path and possibly an inflated map.
- **Key Methods and Classes**:
  - `class GridWorld`: Represents the grid structure for the A* algorithm.
  - `void ProcessQuery()`: Processes incoming pathfinding requests.
  - `Callback` Functions: Handles incoming data from subscribed topics.
  - `void PublishPath_()`: Publishes the calculated path.

#### Overview
This file implements the A* algorithm for pathfinding in a ROS environment. It uses ROS topics for data input and output, and callback functions for asynchronous communication within the ROS network.

---

## pure_pursuit.cpp

### Execution Flow and Main Components
- **Main Function**: 
  - `int main()`: Initializes the ROS node and manages interactions.
- **ROS Topics**:
  - **Subscriptions**: For receiving path data or sensor inputs.
  - **Publications**: For sending out control commands based on the algorithm's output.
- **Key Methods and Classes**:
  - `class PurePursuit`: Encapsulates the Pure Pursuit algorithm logic.
  - `cmd_generator()`: Generates control commands.
  - `waypoints_listener()`: Callback for receiving path data.
  - `checkcase()`, `laser_callback()`: Process environmental data.
  - `run()`: Runs the main process of the algorithm.

#### Overview
`pure_pursuit.cpp` is set up for path following using the Pure Pursuit algorithm in a ROS framework. It demonstrates how ROS nodes can subscribe to data, process it, and publish control commands for robot navigation.

---

You can see the nodes interactiond and the transformations in the images below:

![rosgraph](https://github.com/Gabertho/Trajectory-Planning-and-Autonomous-Navigation-ROS/assets/59297927/47148b49-23a3-4f74-8016-a6674b806d73)

![tftree](https://github.com/Gabertho/Trajectory-Planning-and-Autonomous-Navigation-ROS/assets/59297927/1c802772-8732-42f5-a8a2-f12f4c79da32)




# Results

Regarding the results, we obtain a very interesting autonomous navigation approach, although there are some flaws.
These flaws, which involve coordinate transformation problems in the Pure Pursuit algorithm, and the need to adjust parameters in the Astar algorithm, will be corrected as soon as possible. However, we want to emphasize that the approach has great potential to meet the scope of the project.


[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/JWlF2P1npUk/0.jpg)](https://www.youtube.com/watch?v=JWlF2P1npUk)













