/*********************************************************************
 * Author: Rick Su
 * 2024/7/9
 *********************************************************************/

#include <cmath>
#include <queue>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include "nav2_prm_planner/prm_planner.hpp"

namespace nav2_prm_planner
{
  void PRM::configure(
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

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".max_edge_length", rclcpp::ParameterValue(0.1));
    node_->get_parameter(name_ + ".max_edge_length", _max_edge_length);
    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".max_iter", rclcpp::ParameterValue(10000));
    node_->get_parameter(name_ + ".max_iter", _max_iter);
    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".inflation_multiplier", rclcpp::ParameterValue(1.0));
    node_->get_parameter(name_ + ".inflation_multiplier", _inflation_multiplier);
  }

  void PRM::cleanup()
  {
    RCLCPP_INFO(node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner", name_.c_str());
  }

  void PRM::activate()
  {
    RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type NavfnPlanner", name_.c_str());
  }

  void PRM::deactivate()
  {
    RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner", name_.c_str());
  }

  nav_msgs::msg::Path PRM::createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal)
  {
    auto qos = rclcpp::QoS(5000);
    auto publisher = node_->create_publisher<visualization_msgs::msg::Marker>("/prm", qos);
    publisher->on_activate();

    if (nodes.empty())
    {
      generateRoadmap();
    }

    publishMarkers(publisher, goal);

    nav_msgs::msg::Path global_path;

    if (!validateFrame(start, goal))
    {
      return global_path;
    }

    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    unsigned int startx, starty, goalx, goaly;
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, startx, starty);
    costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goalx, goaly);

    int closest_to_start = findClosestNode(startx, starty);
    int closest_to_goal = findClosestNode(goalx, goaly);

    if (closest_to_start < 0 || closest_to_goal < 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "NO CONNECTION TO GOAL OR START");
      return global_path;
    }

    auto dijkstra = runDijkstra(closest_to_start);

    if (dijkstra[closest_to_goal].parent < 0)
    {
      RCLCPP_INFO(node_->get_logger(), "DIJKSTRA FAILED SADGE");
      return global_path;
    }

    buildPath(global_path, dijkstra, closest_to_goal, goal);

    return global_path;
  }

  void PRM::generateRoadmap()
  {
    for (size_t i = 0; i < _max_iter; i++)
    {
      unsigned int xm = std::rand() % costmap_->getSizeInCellsX();
      unsigned int ym = std::rand() % costmap_->getSizeInCellsY();
      if (costmap_->getCost(xm, ym) > MAX_NON_OBSTACLE)
        continue;

      nodes.push_back({xm, ym});
    }

    edges.resize(nodes.size());

    for (size_t i = 0; i < nodes.size(); i++)
    {
      for (size_t j = 0; j < nodes.size(); j++)
      {
        if (j == i)
          continue;

        auto &p1 = nodes[i];
        auto &p2 = nodes[j];
        float dist = std::hypot(p2.x - p1.x, p2.y - p1.y);

        if (dist > _max_edge_length)
          continue;

        double line_cost = _collision_checker.lineCost(p1.x, p2.x, p1.y, p2.y);
        if (line_cost > MAX_NON_OBSTACLE)
          continue;

        edges[i].emplace_back(j, dist * (1 + _inflation_multiplier * line_cost / 252));
      }
    }
  }

  void PRM::publishMarkers(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &publisher, const geometry_msgs::msg::PoseStamped &goal)
  {
    std_msgs::msg::ColorRGBA color;
    color.a = 1;
    color.r = 0;
    color.g = 0.5;
    color.b = 1;

    visualization_msgs::msg::Marker markerPoint, lineMarker;

    markerPoint.header.frame_id = global_frame_;
    markerPoint.header.stamp = node_->now();
    markerPoint.action = visualization_msgs::msg::Marker::ADD;
    markerPoint.type = visualization_msgs::msg::Marker::POINTS;
    markerPoint.id = 1;
    markerPoint.scale.x = 0.05;
    markerPoint.scale.y = 0.05;
    markerPoint.color = color;

    lineMarker.header.frame_id = global_frame_;
    lineMarker.header.stamp = node_->now();
    lineMarker.action = visualization_msgs::msg::Marker::ADD;
    lineMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
    lineMarker.id = 2;
    lineMarker.scale.x = 0.01;
    lineMarker.color = color;

    for (size_t i = 0; i < nodes.size(); i++)
    {
      double x, y;
      costmap_->mapToWorld(nodes[i].x, nodes[i].y, x, y);
      geometry_msgs::msg::Point point;
      point.x = x;
      point.y = y;
      point.z = goal.pose.position.z;
      markerPoint.points.push_back(point);

      for (auto &&edge : edges[i])
      {
        geometry_msgs::msg::Point pp1, pp2;
        double x1, y1, x2, y2;
        costmap_->mapToWorld(nodes[edge.destination].x, nodes[edge.destination].y, x1, y1);
        costmap_->mapToWorld(nodes[i].x, nodes[i].y, x2, y2);
        pp1.x = x1;
        pp1.y = y1;
        pp1.z = goal.pose.position.z;
        pp2.x = x2;
        pp2.y = y2;
        pp2.z = goal.pose.position.z;
        lineMarker.points.push_back(pp2);
        lineMarker.points.push_back(pp1);
      }
    }

    publisher->publish(lineMarker);
    publisher->publish(markerPoint);
  }

  bool PRM::validateFrame(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal)
  {
    if (start.header.frame_id != global_frame_)
    {
      RCLCPP_ERROR(node_->get_logger(), "Planner will only accept start position from %s frame", global_frame_.c_str());
      return false;
    }

    if (goal.header.frame_id != global_frame_)
    {
      RCLCPP_INFO(node_->get_logger(), "Planner will only accept goal position from %s frame", global_frame_.c_str());
      return false;
    }
    return true;
  }

  int PRM::findClosestNode(unsigned int x, unsigned int y)
  {
    int closest_node = -1;
    float min_dist = std::numeric_limits<float>::max();

    for (size_t i = 0; i < nodes.size(); i++)
    {
      float cur_dist = std::hypot(static_cast<int>(x) - nodes[i].x, static_cast<int>(y) - nodes[i].y);
      if (cur_dist < min_dist && _collision_checker.lineCost(x, nodes[i].x, y, nodes[i].y) <= MAX_NON_OBSTACLE)
      {
        closest_node = i;
        min_dist = cur_dist;
      }
    }
    return closest_node;
  }

  std::vector<Node> PRM::runDijkstra(int start_node)
  {
    std::vector<Node> dijkstra(nodes.size(), {});

    for (size_t i = 0; i < nodes.size(); i++)
    {
      dijkstra[i] = {nodes[i], -1, static_cast<int>(i), std::numeric_limits<float>::max()};
    }

    dijkstra[start_node].distance = 0;
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> tovisit;
    tovisit.emplace(0, start_node);

    while (!tovisit.empty())
    {
      int current = tovisit.top().second;
      tovisit.pop();

      for (auto &&edge : edges[current])
      {
        float new_dist = dijkstra[current].distance + edge.cost;
        if (new_dist < dijkstra[edge.destination].distance)
        {
          dijkstra[edge.destination].distance = new_dist;
          dijkstra[edge.destination].parent = current;
          tovisit.emplace(new_dist, edge.destination);
        }
      }
    }
    return dijkstra;
  }

  void PRM::buildPath(nav_msgs::msg::Path &global_path, const std::vector<Node> &dijkstra, int goal_node, const geometry_msgs::msg::PoseStamped &goal)
  {
    std::vector<geometry_msgs::msg::PoseStamped> temp;
    geometry_msgs::msg::PoseStamped pose = goal;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    temp.push_back(pose);

    for (auto curr = dijkstra[goal_node]; curr.parent != -1; curr = dijkstra[curr.parent])
    {
      double x, y;
      costmap_->mapToWorld(curr.point.x, curr.point.y, x, y);
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      temp.push_back(pose);
    }

    std::reverse(temp.begin(), temp.end());
    global_path.poses = std::move(temp);
  }

} // namespace nav2_rrt_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_prm_planner::PRM, nav2_core::GlobalPlanner)
