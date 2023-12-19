# Trajectory-Planning-and-Autonomous-Navigation-ROS
The repository contains the ROS code for a trajectory planner using the A-star algorithm, localization via AMCL and obstacle detection with LiDAR. It was developed for the “Robo do DC" project at UFSCar, during the Autonomous Mobile Robots discipline in 2023.


# DC ROBOT - UFSCar

This package contains a toolkit designed for the DC Autonomous Mobile Robot. Developed as part of the Department of Computer - Autonomous Mobile Robot (DCAMR) Project, this toolkit provides essential functionalities for the DC robot's autonomous capabilities.

    	
# How to install


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

## IMPORTANT!! You need to change, in the following line, in gazebo.launch file, to your mapa.yaml path.:

```
  <!-- Map Server -->
    <arg name="map_file" default="/home/gabriel/dcrobot_ws/src/dcrobot/mobile_rob_dev_sim/config/mapa.yaml"/>
    <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)"/>
```
Also, in     

# Adopted methodology and results

## Mapping
For the mapping step, we used gmapping [(http://wiki.ros.org/gmapping),](http://wiki.ros.org/gmapping) navigating the simulated environment with teleop. With this, we obtain the map.yaml and map.pgm that represent the environment.

![image](https://github.com/Gabertho/Trajectory-Planning-and-Autonomous-Navigation-ROS/assets/59297927/8fa41d15-c823-42ef-8d07-8d06ff18d384)


## Localization

For localization, we use the ROS amcl package (http://wiki.ros.org/amcl), which implements the Adaptive Monte Carlo Localizer approach. We used this algorithm, instead of using odometry data, as this can accumulate errors throughout use, leading to possible failures in trajectory planning.

![image](https://github.com/Gabertho/Trajectory-Planning-and-Autonomous-Navigation-ROS/assets/59297927/b65c34c0-2cd9-422d-ac97-dfcb9faa28ee)

![image](https://github.com/Gabertho/Trajectory-Planning-and-Autonomous-Navigation-ROS/assets/59297927/affd63af-7249-41c8-a8a0-10b1e3a7c7d0)




## Trajectory Planning
For trajectory planning, we implemented the A* (Astar) algorithm, which calculates the optimal path between the starting and ending points, using a function composed of the cost of the movement plus a heuristic. The heuristic used was Manhattan, as it generates paths that are relatively more "distant from obstacles", such as walls, in relation to the Euclidean distance, which reduces possible collision problems.

The map, obtained in the previous step, is published by the topic /map (package map_server), and used as an Occupancy Grid in the implementation of the A* algorithm (astar.cpp and astar.h).
We used, as base, the Astar code provided on: https://github.com/lh9171338/Astar/tree/master

Pseudocode:

```
function reconstruct_path(cameFrom, current)
    total_path := {current}
    while current in cameFrom.Keys:
        current := cameFrom[current]
        total_path.prepend(current)
    return total_path

// A* finds a path from start to goal.
// h is the heuristic function. h(n) estimates the cost to reach goal from node n.
function A_Star(start, goal, h)
    // The set of discovered nodes that may need to be (re-)expanded.
    // Initially, only the start node is known.
    // This is usually implemented as a min-heap or priority queue rather than a hash-set.
    openSet := {start}

    // For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from the start
    // to n currently known.
    cameFrom := an empty map

    // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore := map with default value of Infinity
    gScore[start] := 0

    // For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    // how cheap a path could be from start to finish if it goes through n.
    fScore := map with default value of Infinity
    fScore[start] := h(start)

    while openSet is not empty
        // This operation can occur in O(Log(N)) time if openSet is a min-heap or a priority queue
        current := the node in openSet having the lowest fScore[] value
        if current = goal
            return reconstruct_path(cameFrom, current)

        openSet.Remove(current)
        for each neighbor of current
            // d(current,neighbor) is the weight of the edge from current to neighbor
            // tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore := gScore[current] + d(current, neighbor)
            if tentative_gScore < gScore[neighbor]
                // This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] := current
                gScore[neighbor] := tentative_gScore
                fScore[neighbor] := tentative_gScore + h(neighbor)
                if neighbor not in openSet
                    openSet.add(neighbor)

    // Open set is empty but goal was never reached
    return failure

```

![image](https://github.com/Gabertho/Trajectory-Planning-and-Autonomous-Navigation-ROS/assets/59297927/6a7401fb-84a1-43f9-a572-e7c15a9afdd7)


We also added the "inflate obstacles" functionality to the A star algorithm, which is the gray region of the print. This prevents trajectories from being drawn close to obstacles such as walls, which could cause the robot to become entangled.


## Navigation 
For navigation, we use an implementation of the Pure Pursuit Path Tracking algorithm. It computes the angular velocity command that moves the robot from its current position to reach some look-ahead point in front of the robot, while the linear velocity is assumed constant. Speed ​​commands are published in the /robot/cmd_vel topic.

We used, as base, the Pure Pursuit implementation provided at: https://github.com/leofansq/ROS_Pure_Pursuit.

Here is the Pure Pursuit basics:

![image](https://github.com/Gabertho/Trajectory-Planning-and-Autonomous-Navigation-ROS/assets/59297927/ba97dad3-22a8-498d-80a7-e57c5ba30f24)


## Obstacle Avoidance
For obstacle avoidance, we implemented a "naive" approach using the LIDAR sensor. The approach consists of stopping the robot's movement on the optimal path when an obstacle is detected, and keeping the robot stationary until the obstacle is no longer detected, thus resuming its trajectory.

![image](https://github.com/Gabertho/Trajectory-Planning-and-Autonomous-Navigation-ROS/assets/59297927/df1ef90d-2d30-4e32-aed5-9baa3d2c7f29)




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

You can see the nodes interaction and the transformations in the images below:

![rosgraph](https://github.com/Gabertho/Trajectory-Planning-and-Autonomous-Navigation-ROS/assets/59297927/47148b49-23a3-4f74-8016-a6674b806d73)

![tftree](https://github.com/Gabertho/Trajectory-Planning-and-Autonomous-Navigation-ROS/assets/59297927/1c802772-8732-42f5-a8a2-f12f4c79da32)



# How to use

- Step 1: roslaunch gazebo.launch
- Step 2: With RVIZ open, set the desired goal  position using the 2D Nav Goal.
- Step 3: It's not necessary to use 2D pose estimate since we are using the AMCL for localization. Just use 2D Nav goal to select the desired destination.
# Results

Regarding the results, we obtain a very interesting autonomous navigation approach, although there are some flaws.
These flaws, which involve coordinate transformation problems in the Pure Pursuit algorithm, and the need to adjust parameters in the Astar algorithm, will be corrected as soon as possible. However, we want to emphasize that the approach has great potential to meet the scope of the project.


[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/OLs-SMkg3II/0.jpg)](https://www.youtube.com/watch?v=OLs-SMkg3II)













