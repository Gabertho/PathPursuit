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



