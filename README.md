# Trajectory-Planning-and-Autonomous-Navigation-ROS
The repository contains the ROS code for a trajectory planner using the A-star algorithm, localization via AMCL and obstacle detection with LiDAR. It was developed for the “Robo do DC" project at UFSCar, during the Autonomous Mobile Robots discipline in 2023.


# DC ROBOT - UFSCar

This package contains a toolkit designed for the DC Autonomous Mobile Robot. Developed as part of the Department of Computer - Autonomous Mobile Robot (DCAMR) Project, this toolkit provides essential functionalities for the DC robot's autonomous capabilities.

    	
# How to use

### Create directories

```bash
mkdir -p /home/$USER/workspace/src
cd /home/$USER/workspace/
```


### Initialize the Catkin workspace
```bash
catkin init
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=Release
```

### Navigate to the directory of `src` to clone the `DC Robot project`

```bash
cd /home/$USER/workspace/src
git clone https://github.com/Gabertho/Trajectory-Planning-and-Autonomous-Navigation-ROS.git
```

### Build the project
```bash
cd /home/$USER/workspace/
catkin build
```

### Source your catkin workspace
```bash
source /home/$USER/workspace/devel/setup.bash
```

### (optional) if the steps above don't work, follow the installation steps at https://github.com/vivaldini/dcrobot and then replace the mobile_robot_dev_sim and envorobotz directories for the ones in this repository.



# Adopted methodology and results

## Mapping
For the mapping step, we used gmapping (http://wiki.ros.org/gmapping), navigating the simulated environment with teleop. With this, we obtain the map.yaml and map.pgm that represent the environment.

## Localization

For localization, we use the ROS amcl package, which implements the Adaptive Monte Carlo Localizer approach. We used this algorithm, instead of using odometry data, as this can accumulate errors throughout use, leading to possible failures in trajectory planning.


## Trajectory Planning
For trajectory planning, we implemented the A* (Astar) algorithm, which calculates the optimal path between the starting and ending points, using a function composed of the cost of the movement plus a heuristic. The heuristic used was Manhattan, as it generates paths that are relatively more "distant from obstacles", such as walls, in relation to the Euclidean distance, which reduces possible collision problems.

The map, obtained in the previous step, is published by the topic /map (package map_server), and used as an Occupancy Grid in the implementation of the A* algorithm (astar.cpp and astar.h)

## Navigation 
For navigation, we use an implementation of the Pure Pursuit Path Tracking algorithm. It computes the angular velocity command that moves the robot from its current position to reach some look-ahead point in front of the robot, while the linear velocity is assumed constant. Speed ​​commands are published in the /robot/cmd_vel topic

## Obstacle Avoidance
For obstacle avoidance, we implemented a "naive" approach using the LIDAR sensor. The approach consists of stopping the robot's movement on the optimal path when an obstacle is detected, and keeping the robot stationary until the obstacle is no longer detected, thus resuming its trajectory.


## Results

Regarding the results, we obtain a very interesting autonomous navigation approach, although there are some flaws.
These flaws, which involve coordinate transformation problems in the Pure Pursuit algorithm, and the need to adjust parameters in the Astar algorithm, will be corrected as soon as possible. However, we want to emphasize that the approach has great potential to meet the scope of the project.


[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/JWlF2P1npUk/0.jpg)](https://www.youtube.com/watch?v=JWlF2P1npUk)






