#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "astar/occupancy_map.h"
#include "astar/astar.h"

#define SUB_TOPIC_NAME_OCCUPANCY_MAP "/map"
#define SUB_TOPIC_NAME_START_STATE "/initialpose"
#define SUB_TOPIC_NAME_TARGET_STATE "/move_base_simple/goal"
#define PUB_TOPIC_NAME_INFLATED_MAP "/inflated_map"
#define PUB_TOPIC_NAME_OPTIMAL_PATH "/optimal_path"
#define DEFAULT_MAP_THRESHOLD_VALUE 65
#define DEFAULT_MAP_INFLATION_RADIUS 0.6 // meter

class GridWorld
{
public:
    GridWorld();
    void ProcessQuery();

private:
    bool map_flag_ = false;
    bool source_point_flag_ = false;
    bool target_point_flag_ = false;
    bool start_planning_flag_ = false;
    tf2::Vector3 source_point_ = {0, 0, 0};
    tf2::Vector3 target_point_ = {0, 0, 0};
    astar::Astar planner_;
    std::shared_ptr<occupancy_map::OccupancyMap> map_ptr_;
    ros::NodeHandle node_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_robot_pose_;
    ros::Subscriber sub_target_point_;
    ros::Publisher pub_path_;
    ros::Publisher pub_inflated_map_;
    void CallbackMap_(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void CallbackTargetPoint_(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void PublishPath_(const std::vector<tf2::Vector3> &path_w);
    void CallbackRobotPose_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg); // ou geometry_msgs::PoseWithCovarianceStamped p/ amcl
};

GridWorld::GridWorld()
{
    // Initilize subscribers and publishers
    sub_map_ = node_.subscribe(SUB_TOPIC_NAME_OCCUPANCY_MAP, 1, &GridWorld::CallbackMap_, this);
    sub_robot_pose_ = node_.subscribe("/amcl_pose", 1, &GridWorld::CallbackRobotPose_, this); // ou o tópico apropriado
    sub_target_point_ = node_.subscribe(SUB_TOPIC_NAME_TARGET_STATE, 1, &GridWorld::CallbackTargetPoint_, this);
    pub_inflated_map_ = node_.advertise<nav_msgs::OccupancyGrid>(PUB_TOPIC_NAME_INFLATED_MAP, 1, this);
    pub_path_ = node_.advertise<nav_msgs::Path>(PUB_TOPIC_NAME_OPTIMAL_PATH, 1, this);

    // Get parameters
    ros::NodeHandle node_prv("~"); // private node for private params
    int occupancy_map_threshold;
    double occupancy_map_inflatiion_radius;
    node_prv.param<int>("occupancy_map_threshold", occupancy_map_threshold, DEFAULT_MAP_THRESHOLD_VALUE);
    node_prv.param<double>("occupancy_map_inflation_radius", occupancy_map_inflatiion_radius, DEFAULT_MAP_INFLATION_RADIUS);

    // Create an occupancy map object with parameters
    map_ptr_ = std::make_shared<occupancy_map::OccupancyMap>(occupancy_map::OccupancyMap(occupancy_map_threshold, occupancy_map_inflatiion_radius));

    // Initiate planner
    planner_ = astar::Astar(map_ptr_);
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

void GridWorld::CallbackRobotPose_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    // Atualize source_point_ com a posição atual do robô
    source_point_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    // Ajuste as flags conforme necessário
    source_point_flag_ = true;
    if (map_flag_ && target_point_flag_)
    {
        start_planning_flag_ = true;
    }
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
    target_point_flag_ = false;
    start_planning_flag_ = false;
}

void GridWorld::CallbackTargetPoint_(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    target_point_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    ROS_INFO("Received target point: x = %f, y = %f", target_point_.x(), target_point_.y());

    target_point_flag_ = true;
    if (map_flag_ && source_point_flag_)
    {
        start_planning_flag_ = true;
        ROS_INFO("Target pose received, ready to start planning.");
    }
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
