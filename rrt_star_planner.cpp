#include <cmath>
#include <vector>
#include <random>
#include <memory>
#include <limits>
#include "nav2_util/node_utils.hpp"
#include "nav2_straightline_planner/rrt_star_planner.hpp"

namespace nav2_rrt_star_planner
{

struct Node {
  geometry_msgs::msg::PoseStamped pose;
  Node* parent;
  double cost;
};

class RRTStar : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".max_iterations", rclcpp::ParameterValue(1000));
    node_->get_parameter(name_ + ".max_iterations", max_iterations_);
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    nav_msgs::msg::Path global_path;
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    if (start.header.frame_id != global_frame_) {
      RCLCPP_ERROR(node_->get_logger(), "Start position must be in %s frame", global_frame_.c_str());
      return global_path;
    }
    if (goal.header.frame_id != global_frame_) {
      RCLCPP_ERROR(node_->get_logger(), "Goal position must be in %s frame", global_frame_.c_str());
      return global_path;
    }

    std::vector<Node*> nodes;
    Node* start_node = new Node{start, nullptr, 0.0};
    nodes.push_back(start_node);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInMetersX());
    std::uniform_real_distribution<> dis_y(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInMetersY());

    for (int i = 0; i < max_iterations_; ++i) {
      geometry_msgs::msg::PoseStamped rand_pose;
      rand_pose.pose.position.x = dis_x(gen);
      rand_pose.pose.position.y = dis_y(gen);

      Node* nearest_node = nullptr;
      double min_dist = std::numeric_limits<double>::max();
      for (auto& node : nodes) {
        double dist = std::hypot(rand_pose.pose.position.x - node->pose.pose.position.x,
                                 rand_pose.pose.position.y - node->pose.pose.position.y);
        if (dist < min_dist) {
          min_dist = dist;
          nearest_node = node;
        }
      }

      if (nearest_node) {
        double theta = std::atan2(rand_pose.pose.position.y - nearest_node->pose.pose.position.y,
                                  rand_pose.pose.position.x - nearest_node->pose.pose.position.x);
        Node* new_node = new Node{
          geometry_msgs::msg::PoseStamped(), nearest_node,
          nearest_node->cost + min_dist
        };
        new_node->pose.pose.position.x = nearest_node->pose.pose.position.x + std::cos(theta) * min_dist;
        new_node->pose.pose.position.y = nearest_node->pose.pose.position.y + std::sin(theta) * min_dist;

        if (isValid(new_node->pose.pose.position.x, new_node->pose.pose.position.y)) {
          nodes.push_back(new_node);
          if (std::hypot(new_node->pose.pose.position.x - goal.pose.position.x,
                         new_node->pose.pose.position.y - goal.pose.position.y) < 0.5) {
            goal_pose_ = new_node;
            break;
          }
        }
      }
    }

    Node* node = goal_pose_;
    while (node) {
      global_path.poses.insert(global_path.poses.begin(), node->pose);
      node = node->parent;
    }

    return global_path;
  }

private:
  bool isValid(double x, double y)
  {
    unsigned int mx, my;
    costmap_->worldToMap(x, y, mx, my);
    return costmap_->getCost(mx, my) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  rclcpp::Node::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D* costmap_;
  std::string global_frame_;
  int max_iterations_;
  Node* goal_pose_;
};

}  // namespace nav2_rrt_star_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrt_star_planner::RRTStar, nav2_core::GlobalPlanner)
