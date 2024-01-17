#include <chrono>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>

#include "astar/occupancy_map.h"
#include "astar/astar.h"

#define SUB_TOPIC_NAME_OCCUPANCY_MAP "/map"
#define SUB_TOPIC_NAME_START_STATE "/initialpose"
#define PUB_TOPIC_NAME_INFLATED_MAP "/inflated_map"
#define PUB_TOPIC_NAME_OPTIMAL_PATH "/optimal_path"
#define PUB_TOPIC_NAME_SETWAYPOINT "/set_waypoint"
#define DEFAULT_MAP_THRESHOLD_VALUE 65
#define DEFAULT_MAP_INFLATION_RADIUS 1.0 // meter

class GridWorld
{
public:
    GridWorld();
    void ProcessQuery();
    void SetTargetWaypoint(const std::string &waypoint_name);

private:
    bool map_flag_ = false;
    bool source_point_flag_ = false;
    bool start_planning_flag_ = false;
    tf2::Vector3 source_point_ = {0, 0, 0};
    tf2::Vector3 target_point_ = {0, 0, 0};
    std::string target_waypoint_name_;
    astar::Astar planner_;
    std::shared_ptr<occupancy_map::OccupancyMap> map_ptr_;
    ros::NodeHandle node_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_waypoint_;
    ros::Subscriber sub_robot_pose_;
    ros::Publisher pub_path_;
    ros::Publisher pub_waypoint_;
    ros::Publisher pub_inflated_map_;
    std::map<std::string, tf2::Vector3> waypoints_map_;
    void CallbackMap_(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void PublishPath_(const std::vector<tf2::Vector3> &path_w);
    void CallbackRobotPose_(const nav_msgs::Odometry::ConstPtr &msg);
    void CallbackWaypoint_(const std_msgs::String::ConstPtr &msg);
};

GridWorld::GridWorld()
{
    // Initialize subscribers and publishers
    sub_map_ = node_.subscribe(SUB_TOPIC_NAME_OCCUPANCY_MAP, 1, &GridWorld::CallbackMap_, this);
    sub_robot_pose_ = node_.subscribe("/odom", 1, &GridWorld::CallbackRobotPose_, this);
    sub_waypoint_ = node_.subscribe("/set_waypoint", 1, &GridWorld::CallbackWaypoint_, this);
    pub_inflated_map_ = node_.advertise<nav_msgs::OccupancyGrid>(PUB_TOPIC_NAME_INFLATED_MAP, 1, this);
    pub_path_ = node_.advertise<nav_msgs::Path>(PUB_TOPIC_NAME_OPTIMAL_PATH, 1, this);
    pub_waypoint_ = node_.advertise<std_msgs::String>(PUB_TOPIC_NAME_SETWAYPOINT, 1, this);

    // Get parameters
    ros::NodeHandle node_prv("~"); // private node for private params
    int occupancy_map_threshold;
    double occupancy_map_inflation_radius;
    node_prv.param<int>("occupancy_map_threshold", occupancy_map_threshold, DEFAULT_MAP_THRESHOLD_VALUE);
    node_prv.param<double>("occupancy_map_inflation_radius", occupancy_map_inflation_radius, DEFAULT_MAP_INFLATION_RADIUS);

    // Create an occupancy map object with parameters
    map_ptr_ = std::make_shared<occupancy_map::OccupancyMap>(occupancy_map::OccupancyMap(occupancy_map_threshold, occupancy_map_inflation_radius));

    // Initialize planner
    planner_ = astar::Astar(map_ptr_);

    // Define waypoints
    waypoints_map_ = {
        {"LE-1", {-37.99, -5.45, 0.0}},
        {"LE-2", {-30.15, -5.03, 0.0}},
        {"LE-3", {-22.68, -4.45, 0.0}},
        {"LE-4", {-15.36, -4.11, 0.0}},
        // ... add more waypoints as needed
    };
}

void GridWorld::SetTargetWaypoint(const std::string &waypoint_name)
{
    auto it = waypoints_map_.find(waypoint_name);
    if (it != waypoints_map_.end())
    {
        target_waypoint_name_ = waypoint_name;
        target_point_ = it->second;
        ROS_INFO("Target waypoint set to: %s", waypoint_name.c_str());
        start_planning_flag_ = true;
    }
    else
    {
        ROS_WARN("Waypoint %s not found.", waypoint_name.c_str());
    }
}

void GridWorld::ProcessQuery()
{
    if (start_planning_flag_ == true)
    {
        ROS_INFO("Starting path planning.");
        std::vector<tf2::Vector3> path_w;
        std::chrono::_V2::system_clock::time_point start, stop;

        // Get beginning of planning time
        start = std::chrono::high_resolution_clock::now();

        // Plan
        planner_.Plan(source_point_, target_point_, path_w);

        // Get end of planning time
        stop = std::chrono::high_resolution_clock::now();

        // Calculate duration of planning
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        // Inform the user
        ROS_INFO("Planning is done in %.4f secs.", (double)duration.count() / 1e6);

        if (!path_w.empty())
        {
            PublishPath_(path_w);
        }
        else
        {
            ROS_WARN("Path planning failed to find a path.");
        }

        start_planning_flag_ = false;
    }
    else
    {
        ROS_DEBUG("Waiting for planning conditions to be met.");
    }
}

void GridWorld::CallbackRobotPose_(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Update source_point_ with the current position of the robot
    source_point_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0};
    // Adjust flags as necessary
    source_point_flag_ = true;
}

void GridWorld::CallbackMap_(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    // Update the map object
    map_ptr_->UpdateMap(msg);

    // Update planner
    planner_.InitializeGraph();

    // Publish inflated map for rviz to visualize
    nav_msgs::OccupancyGrid inflated_map_msg;
    inflated_map_msg.header.frame_id = map_ptr_->GetFrameID();
    inflated_map_msg.header.stamp = ros::Time::now();
    map_ptr_->GetInflatedMapMsg(&inflated_map_msg);
    pub_inflated_map_.publish(inflated_map_msg);

    // Set flags
    map_flag_ = true;
    source_point_flag_ = false;
}

void GridWorld::CallbackWaypoint_(const std_msgs::String::ConstPtr &msg)
{
    SetTargetWaypoint(msg->data);
    ProcessQuery();
}

void GridWorld::PublishPath_(const std::vector<tf2::Vector3> &path_w)
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = map_ptr_->GetFrameID();
    path_msg.header.stamp = ros::Time::now();

    for (const auto &point_w : path_w)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = point_w.x();
        pose.pose.position.y = point_w.y();
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    pub_path_.publish(path_msg);
    ROS_INFO("Path published with %ld points.", path_w.size());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar_node");
    GridWorld env;
    ros::Rate rate(20);
    while (ros::ok())
    {
        env.ProcessQuery();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
