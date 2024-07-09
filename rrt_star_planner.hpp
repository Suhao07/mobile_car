#ifndef NAV2_RRT_STAR_PLANNER__RRT_STAR_PLANNER_HPP_
#define NAV2_RRT_STAR_PLANNER__RRT_STAR_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_rrt_star_planner
{

struct Node {
  geometry_msgs::msg::PoseStamped pose;
  Node* parent;
  double cost;

  Node(const geometry_msgs::msg::PoseStamped& p, Node* par = nullptr, double c = 0.0)
  : pose(p), parent(par), cost(c) {}
};

class RRTStar : public nav2_core::GlobalPlanner
{
public:
  RRTStar() = default;
  ~RRTStar() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  bool isValid(double x, double y);
  double distance(const geometry_msgs::msg::PoseStamped & p1, const geometry_msgs::msg::PoseStamped & p2);
  Node* getNearestNode(const geometry_msgs::msg::PoseStamped & pose);
  void rewire(Node* new_node, const std::vector<Node*> & nearby_nodes);

  rclcpp::Node::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D* costmap_;
  std::string global_frame_;
  int max_iterations_;
  double search_radius_;
  Node* goal_pose_;

  std::vector<Node*> nodes_;
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<> dis_x_;
  std::uniform_real_distribution<> dis_y_;
};

}  // namespace nav2_rrt_star_planner

#endif  // NAV2_RRT_STAR_PLANNER__RRT_STAR_PLANNER_HPP_
