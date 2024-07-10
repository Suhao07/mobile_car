#include <cmath>
#include <string>
#include <memory>
#include <tuw_graph_msgs/msg/graph.hpp>
#include <tuw_graph/search_astar.hpp>
#include <tuw_graph/ros_bridge.hpp>
#include "nav2_util/node_utils.hpp"
#include "tuw_planner_graph/graph_astar.hpp"

using std::placeholders::_1;

namespace tuw_planner_graph
{

void GraphAStar::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
    node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void GraphAStar::cleanup()
{
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type NavfnAStarGraphPlanner",
        name_.c_str());
}

void GraphAStar::activate()
{
    sub_graph_ = node_->create_subscription<tuw_graph_msgs::msg::Graph>(
        "/graph", 10, std::bind(&GraphAStar::callback_graph, this, _1));

    pub_path_debug_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("path_debug", 10);

    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type NavfnAStarGraphPlanner",
        name_.c_str());
}

void GraphAStar::deactivate()
{
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type NavfnAStarGraphPlanner",
        name_.c_str());
}

nav_msgs::msg::Path &GraphAStar::plan_graph_astar(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    nav_msgs::msg::Path &global_path)
{
    // Transform start and goal to graph coordinates
    auto start_map = poseToVector(start.pose.position);
    auto goal_map = poseToVector(goal.pose.position);
    auto tf_map_2_graph = graph_->origin().inverse();
    auto tf_graph_2_map = graph_->origin();

    // Find closest graph nodes
    auto node_start = graph_->closest_node(start_map);
    auto node_goal = graph_->closest_node(goal_map);

    // Run A* algorithm
    tuw_graph::SearchAStart astar(*graph_);
    auto path = astar.start_processing(node_start, node_goal, false);
    std::reverse(path.begin(), path.end());

    // Debug message with path
    if (true)
    {
        publishDebugPath(path);
    }

    // Build global path
    if (!path.empty())
    {
        auto [nr_of_interpolations, x_increment, y_increment] = computeInterpolations(path, start_map);
        buildGlobalPath(start, path, global_path, nr_of_interpolations, x_increment, y_increment);
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "Path empty!!!.");
    }

    // Append goal pose to global path
    auto goal_pose = goal;
    appendGoalPose(global_path, goal_pose);

    return global_path;
}

nav_msgs::msg::Path GraphAStar::createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal)
{
    nav_msgs::msg::Path global_path;

    RCLCPP_INFO(
        node_->get_logger(), "start <%4.3f, %4.3f, %4.3f> --> goal  <%4.3f, %4.3f, %4.3f> ",
        start.pose.position.x, start.pose.position.y, start.pose.position.z,
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

    if (msg_graph_)
    {
        plan_graph_astar(start, goal, global_path);
    }
    else
    {
        RCLCPP_WARN(
            node_->get_logger(), "Panner can't start! No graph for planning received.");
    }

    return global_path;
}

void GraphAStar::callback_graph(const tuw_graph_msgs::msg::Graph::SharedPtr msg)
{
    msg_graph_ = std::make_shared<tuw_graph_msgs::msg::Graph>();
    *msg_graph_ = *msg;

    auto graph = std::make_shared<tuw_graph::Graph>();
    tuw_graph::from_msg(*msg_graph_, *graph);

    graph_ = graph;

    RCLCPP_INFO(
        node_->get_logger(), "Graph received %zu edges %zu nodes", msg_graph_->edges.size(), msg_graph_->nodes.size());
}

void GraphAStar::publishDebugPath(const std::vector<tuw_graph::Node *> &path)
{
    std::stringstream ss;
    geometry_msgs::msg::PoseArray path_debug;
    path_debug.header.stamp = node_->now();
    path_debug.header.frame_id = global_frame_;
    auto tf_graph_2_map = graph_->origin();
    for (size_t i = 0; i < path.size(); i++)
    {
        path_debug.poses.push_back(tuw_graph::to_msg(tf_graph_2_map * path[i]->pose));
        ss << path[i]->id << (i < path.size() - 1 ? " > " : "");
    }
    pub_path_debug_->publish(path_debug);
    RCLCPP_INFO(node_->get_logger(), "nodes: %s", ss.str().c_str());
}

std::tuple<int, double, double> GraphAStar::computeInterpolations(
    const std::vector<tuw_graph::Node *> &path, const Eigen::Vector3d &start_map)
{
    int nr_of_interpolations = 0;
    double x_increment = 0;
    double y_increment = 0;
    auto tf_graph_2_map = graph_->origin();
    while (nr_of_interpolations == 0 && path.size() > 0)
    {
        path.erase(path.begin()); // erase first node
        auto first_node_map = tf_graph_2_map * path.front()->pose.translation();
        nr_of_interpolations = (first_node_map - start_map).norm() / interpolation_resolution_;
        x_increment = (first_node_map[0] - start_map[0]) / nr_of_interpolations;
        y_increment = (first_node_map[1] - start_map[1]) / nr_of_interpolations;
    }
    return {nr_of_interpolations, x_increment, y_increment};
}

void GraphAStar::buildGlobalPath(
    const geometry_msgs::msg::PoseStamped &start,
    const std::vector<tuw_graph::Node *> &path,
    nav_msgs::msg::Path &global_path,
    int nr_of_interpolations,
    double x_increment,
    double y_increment)
{
    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    auto pose = start;
    for (int i = 1; i < nr_of_interpolations; ++i)
    {
        pose.pose.position.x = start.pose.position.x + x_increment * i;
        pose.pose.position.y = start.pose.position.y + y_increment * i;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        global_path.poses.push_back(pose);
    }

    auto tf_graph_2_map = graph_->origin();
    for (size_t i = 1; i < path.size(); i++)
    {
        auto node_map = tf_graph_2_map * path[i]->pose.translation();
        pose.pose.position.x = node_map.x();
        pose.pose.position.y = node_map.y();
        pose.pose.position.z = 0.0;
        global_path.poses.push_back(pose);
    }
}

void GraphAStar::appendGoalPose(nav_msgs::msg::Path &global_path, geometry_msgs::msg::PoseStamped &goal_pose)
{
    goal_pose.header.stamp = node_->now();
    goal_pose.header.frame_id = global_frame_;
    global_path.poses.push_back(std::move(goal_pose));
}

Eigen::Vector3d GraphAStar::poseToVector(const geometry_msgs::msg::Point &position)
{
    return Eigen::Vector3d(position.x, position.y, 0);
}

} // namespace tuw_planner_graph

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(tuw_planner_graph::GraphAStar, nav2_core::GlobalPlanner)
