/*********************************************************************
Created by Rick Su 2024/7/9
 *********************************************************************/

#ifndef NAV2_RRT_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_RRT_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_rrt_planner
{
  const float UNKNOWN = 255.0;
  const float OCCUPIED = 254.0;
  const float INSCRIBED = 253.0;
  const float MAX_NON_OBSTACLE = 252.0;
  const float FREE = 0;

  class RRT : public nav2_core::GlobalPlanner
  {
  public:
    RRT();
    ~RRT() = default;

    // plugin configure
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
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
    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;

    // node ptr
    nav2_util::LifecycleNode::SharedPtr node_;

    // Global Costmap
    nav2_costmap_2d::Costmap2D *costmap_;

    // The global frame of the costmap
    std::string global_frame_, name_;

    double RRT_length;
    int _max_iter;
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> _collision_checker;

    // Private methods for improved code structure and readability
    bool validateFrame(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal);
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr setupVisualization();
    std::shared_ptr<nav2_rrt_planner::Node> createNode(double x, double y, nav2_rrt_planner::Node *parent = nullptr);
    std::pair<double, double> generateRandomPoint();
    int findNearestNeighbour(const std::vector<std::shared_ptr<nav2_rrt_planner::Node>> &point_list, double x, double y);
    std::pair<double, double> clipToStepSize(const std::shared_ptr<nav2_rrt_planner::Node> &nearest_node, double x, double y);
    bool isCollision(const std::shared_ptr<nav2_rrt_planner::Node> &nearest_node, double x, double y);
    void visualizeNode(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &publisher, double x, double y);
    void visualizeEdge(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &publisher, double x1, double y1, double x2, double y2);
    bool isGoalReached(const geometry_msgs::msg::PoseStamped &goal, double x, double y);
    nav_msgs::msg::Path buildPath(nav_msgs::msg::Path &global_path, std::shared_ptr<nav2_rrt_planner::Node> &node, const geometry_msgs::msg::PoseStamped &goal);
  };

  class Node
  {
  public:
    Node() = default;
    Node(double x, double y, Node *parent = nullptr) : x(x), y(y), parent(parent) {}
    ~Node() { parent = nullptr; }

    double x;
    double y;
    Node *parent;
  };

} // namespace nav2_rrt_planner

#endif // NAV2_RRT_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
