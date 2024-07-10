#ifndef TUW_PLANNER_GRAPH__GRAPH_ASTAR_HPP_
#define TUW_PLANNER_GRAPH__GRAPH_ASTAR_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tuw_graph/ros_bridge.hpp"

namespace tuw_planner_graph
{

class GraphAStar : public nav2_core::GlobalPlanner
{
public:
    GraphAStar();
    ~GraphAStar() override;

    // plugin configure
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    // plugin cleanup
    void cleanup() override;

    // plugin activate
    void activate() override;

    // plugin deactivate
    void deactivate() override;

    // This method creates path for given start and goal pose.
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) override;

private:
    nav_msgs::msg::Path &plan_graph_astar(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal,
        nav_msgs::msg::Path &global_path);

    // callback on graph msg
    void callback_graph(const tuw_graph_msgs::msg::Graph::SharedPtr msg);

    // ROS subscriptions and publishers
    rclcpp::Subscription<tuw_graph_msgs::msg::Graph>::SharedPtr sub_graph_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_path_debug_;

    // Graph and TF
    tuw_graph_msgs::msg::Graph::SharedPtr msg_graph_;
    tuw_graph::GraphPtr graph_;
    std::shared_ptr<tf2_ros::Buffer> tf_;

    // Node and Costmap
    nav2_util::LifecycleNode::SharedPtr node_;
    nav2_costmap_2d::Costmap2D *costmap_;
    std::string global_frame_, name_;

    // Interpolation parameter
    double interpolation_resolution_;
};

} // namespace tuw_planner_graph

#endif // TUW_PLANNER_GRAPH__GRAPH_ASTAR_HPP_
