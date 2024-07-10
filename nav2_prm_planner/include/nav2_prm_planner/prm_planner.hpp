/*********************************************************************
 * Author: Rick Su 
 * 2024/7/9
 *********************************************************************/

#ifndef NAV2_PRM_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_PRM_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace nav2_prm_planner
{
  const float UNKNOWN = 255.0;
  const float OCCUPIED = 254.0;
  const float INSCRIBED = 253.0;
  const float MAX_NON_OBSTACLE = 252.0;
  const float FREE = 0;

  struct Point
  {
    int x;
    int y;
  };

  struct Edge
  {
    unsigned int destination;
    float cost;
  };

  struct Node
  {
    nav2_prm_planner::Point point;
    int parent;
    int index;
    float distance;
  };

  class PRM : public nav2_core::GlobalPlanner
  {
  public:
    PRM() = default;
    ~PRM() = default;

    // Plugin configuration
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    // Plugin cleanup
    void cleanup() override;

    // Plugin activation
    void activate() override;

    // Plugin deactivation
    void deactivate() override;

    // Create path for given start and goal pose
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) override;

  private:
    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;

    // Node pointer
    nav2_util::LifecycleNode::SharedPtr node_;

    // Global Costmap
    nav2_costmap_2d::Costmap2D *costmap_;

    // The global frame of the costmap
    std::string global_frame_, name_;

    double _max_edge_length;
    int _max_iter;
    float _inflation_multiplier;
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> _collision_checker;

    std::vector<nav2_prm_planner::Point> nodes;
    std::vector<std::vector<nav2_prm_planner::Edge>> edges;

    // Helper functions
    void generateRoadmap();
    void publishMarkers(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &publisher, const geometry_msgs::msg::PoseStamped &goal);
    bool validateFrame(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal);
    int findClosestNode(unsigned int x, unsigned int y);
    std::vector<Node> runDijkstra(int start_node);
    void buildPath(nav_msgs::msg::Path &global_path, const std::vector<Node> &dijkstra, int goal_node, const geometry_msgs::msg::PoseStamped &goal);
  };

} // namespace nav2_prm_planner

#endif // NAV2_PRM_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
