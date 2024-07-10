/*********************************************************************
 Created by Rick Su 2024/7/10
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "nav2_rrt_planner/rrt_planner.hpp"

namespace nav2_rrt_planner
{
  RRT::RRT()
      : costmap_(nullptr),
        tf_(nullptr)
  {
  }

  void RRT::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
    _collision_checker = nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>(costmap_);
    _collision_checker.setCostmap(costmap_);

    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".RRT_step_length", rclcpp::ParameterValue(0.1));
    node_->get_parameter(name_ + ".RRT_step_length", RRT_length);
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".max_iter", rclcpp::ParameterValue(10000));
    node_->get_parameter(name_ + ".max_iter", _max_iter);
  }

  void RRT::cleanup()
  {
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void RRT::activate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void RRT::deactivate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  nav_msgs::msg::Path RRT::createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal)
  {
    nav_msgs::msg::Path global_path;
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    if (!validateFrame(start, goal))
    {
      return global_path;
    }

    auto publisher = setupVisualization();
    auto start_node = createNode(start.pose.position.x, start.pose.position.y);
    std::vector<std::shared_ptr<nav2_rrt_planner::Node>> point_list = {start_node};

    RCLCPP_INFO(
        node_->get_logger(), "Starting path making. Start: X%f Y%f | Goal: X%f Y%f", 
        start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    for (size_t i = 0; i < _max_iter; i++)
    {
      auto [x, y] = generateRandomPoint();
      auto nearest_neighbour = findNearestNeighbour(point_list, x, y);

      auto [new_x, new_y] = clipToStepSize(point_list[nearest_neighbour], x, y);
      if (!isCollision(point_list[nearest_neighbour], new_x, new_y))
      {
        auto new_node = createNode(new_x, new_y, point_list[nearest_neighbour]);
        point_list.push_back(new_node);

        visualizeNode(publisher, new_x, new_y);
        visualizeEdge(publisher, new_x, new_y, point_list[nearest_neighbour]->x, point_list[nearest_neighbour]->y);

        if (isGoalReached(goal, new_x, new_y))
        {
          return buildPath(global_path, new_node, goal);
        }
      }
    }

    RCLCPP_INFO(
        node_->get_logger(), "NO PATH !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    return global_path;
  }

  bool RRT::validateFrame(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal)
  {
    if (start.header.frame_id != global_frame_)
    {
      RCLCPP_ERROR(
          node_->get_logger(), "Planner will only accept start position from %s frame",
          global_frame_.c_str());
      return false;
    }

    if (goal.header.frame_id != global_frame_)
    {
      RCLCPP_INFO(
          node_->get_logger(), "Planner will only accept goal position from %s frame",
          global_frame_.c_str());
      return false;
    }

    return true;
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr RRT::setupVisualization()
  {
    auto qos = rclcpp::QoS(1000);
    auto publisher = node_->create_publisher<visualization_msgs::msg::Marker>("/rrt", qos);
    publisher->on_activate();
    return publisher;
  }

  std::shared_ptr<nav2_rrt_planner::Node> RRT::createNode(double x, double y, nav2_rrt_planner::Node *parent)
  {
    auto node = std::make_shared<nav2_rrt_planner::Node>();
    node->x = x;
    node->y = y;
    node->parent = parent;
    return node;
  }

  std::pair<double, double> RRT::generateRandomPoint()
  {
    unsigned int xm = (std::rand() % costmap_->getSizeInCellsX());
    unsigned int ym = (std::rand() % costmap_->getSizeInCellsY());
    double x, y;
    costmap_->mapToWorld(xm, ym, x, y);
    return {x, y};
  }

  int RRT::findNearestNeighbour(const std::vector<std::shared_ptr<nav2_rrt_planner::Node>> &point_list, double x, double y)
  {
    int nearest_neighbour = 0;
    double smallest_dist = std::numeric_limits<double>::max();
    for (size_t j = 0; j < point_list.size(); j++)
    {
      double dist = std::hypot(x - point_list[j]->x, y - point_list[j]->y);
      if (dist < smallest_dist)
      {
        smallest_dist = dist;
        nearest_neighbour = j;
      }
    }
    return nearest_neighbour;
  }

  std::pair<double, double> RRT::clipToStepSize(const std::shared_ptr<nav2_rrt_planner::Node> &nearest_node, double x, double y)
  {
    double delta_x = x - nearest_node->x;
    double delta_y = y - nearest_node->y;
    double dist = std::hypot(delta_x, delta_y);

    if (dist > RRT_length)
    {
      x = nearest_node->x + RRT_length * delta_x / dist;
      y = nearest_node->y + RRT_length * delta_y / dist;
    }

    return {x, y};
  }

  bool RRT::isCollision(const std::shared_ptr<nav2_rrt_planner::Node> &nearest_node, double x, double y)
  {
    unsigned int nearest_xm, nearest_ym, xm, ym;
    costmap_->worldToMap(nearest_node->x, nearest_node->y, nearest_xm, nearest_ym);
    costmap_->worldToMap(x, y, xm, ym);
    return _collision_checker.lineCost(nearest_xm, xm, nearest_ym, ym) > MAX_NON_OBSTACLE;
  }

  void RRT::visualizeNode(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &publisher, double x, double y)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = node_->now();
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;

    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = 0.0;
    marker.points.push_back(point);

    publisher->publish(marker);
  }

  void RRT::visualizeEdge(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &publisher, double x1, double y1, double x2, double y2)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = node_->now();
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;

    geometry_msgs::msg::Point p1, p2;
    p1.x = x1;
    p1.y = y1;
    p1.z = 0.0;
    p2.x = x2;
    p2.y = y2;
    p2.z = 0.0;

    marker.points.push_back(p1);
    marker.points.push_back(p2);

    publisher->publish(marker);
  }

  bool RRT::isGoalReached(const geometry_msgs::msg::PoseStamped &goal, double x, double y)
  {
    return std::hypot(goal.pose.position.x - x, goal.pose.position.y - y) <= RRT_length;
  }

  nav_msgs::msg::Path RRT::buildPath(nav_msgs::msg::Path &global_path, std::shared_ptr<nav2_rrt_planner::Node> &node, const geometry_msgs::msg::PoseStamped &goal)
  {
    geometry_msgs::msg::PoseStamped pose = goal;
    global_path.poses.push_back(pose);

    auto current_node = node;
    while (current_node != nullptr)
    {
      pose.pose.position.x = current_node->x;
      pose.pose.position.y = current_node->y;
      global_path.poses.push_back(pose);
      current_node = current_node->parent;
    }

    std::reverse(global_path.poses.begin(), global_path.poses.end());

    RCLCPP_INFO(node_->get_logger(), "PATH FOUND !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    return global_path;
  }

} // namespace nav2_rrt_planner

PLUGINLIB_EXPORT_CLASS(nav2_rrt_planner::RRT, nav2_core::GlobalPlanner)
