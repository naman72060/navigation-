#include <cmath>
#include <string>
#include <memory>
#include <queue>
#include <vector>
#include <algorithm>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_custom_planner/custom_planner.hpp"

namespace nav2_custom_planner
{

void CustomPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void CustomPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s of type CustomPlanner", name_.c_str());
}

void CustomPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type CustomPlanner", name_.c_str());
}

void CustomPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type CustomPlanner", name_.c_str());
}

nav_msgs::msg::Path CustomPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(node_->get_logger(), "Start and goal must be in the %s frame", global_frame_.c_str());
    return global_path;
  }

  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    RCLCPP_ERROR(node_->get_logger(), "Start or goal is out of costmap bounds.");
    return global_path;
  }

  const int width = costmap_->getSizeInCellsX();
  const int height = costmap_->getSizeInCellsY();

  struct Node {
    int x, y;
    double g, h;
    Node* parent;
    double f() const { return g + h; }
    
    bool operator<(const Node& other) const {
      return f() > other.f(); // For min-heap
    }
  };

  auto heuristic = [&](int x, int y) {
    return std::hypot(mx_goal - x, my_goal - y);
  };

  // Use map to track best g-cost for each cell
  std::vector<std::vector<double>> g_costs(width, std::vector<double>(height, std::numeric_limits<double>::infinity()));
  std::vector<std::vector<Node*>> node_map(width, std::vector<Node*>(height, nullptr));
  
  std::priority_queue<Node> open_list;
  std::vector<std::unique_ptr<Node>> all_nodes; // For memory management

  auto createNode = [&](int x, int y, double g, Node* parent) -> Node* {
    all_nodes.push_back(std::make_unique<Node>(Node{x, y, g, heuristic(x, y), parent}));
    return all_nodes.back().get();
  };

  Node* start_node = createNode(mx_start, my_start, 0.0, nullptr);
  open_list.push(*start_node);
  g_costs[mx_start][my_start] = 0.0;
  node_map[mx_start][my_start] = start_node;

  Node* final_node = nullptr;
  const std::vector<std::pair<int, int>> directions = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {-1, -1}, {1, -1}, {-1, 1}
  };

  while (!open_list.empty()) {
    Node current = open_list.top();
    open_list.pop();

    if (current.x == (int)mx_goal && current.y == (int)my_goal) {
      final_node = node_map[current.x][current.y];
      break;
    }

    // Skip if we've already found a better path to this node
    if (current.g > g_costs[current.x][current.y]) {
      continue;
    }

    for (const auto& [dx, dy] : directions) {
      int nx = current.x + dx;
      int ny = current.y + dy;

      if (nx < 0 || ny < 0 || nx >= width || ny >= height)
        continue;

      unsigned char cost = costmap_->getCost(nx, ny);
      if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        continue;

      double step_cost = std::hypot(dx, dy) + cost / 255.0;
      double tentative_g = current.g + step_cost;

      // Skip if we've already found a better path to this neighbor
      if (tentative_g >= g_costs[nx][ny]) {
        continue;
      }

      Node* neighbor = createNode(nx, ny, tentative_g, node_map[current.x][current.y]);
      g_costs[nx][ny] = tentative_g;
      node_map[nx][ny] = neighbor;
      open_list.push(*neighbor);
    }
  }

  if (!final_node) {
    RCLCPP_WARN(node_->get_logger(), "A* failed to find a path.");
    return global_path;
  }

  // Reconstruct path
  std::vector<Node*> path;
  Node* current = final_node;
  while (current != nullptr) {
    path.push_back(current);
    current = current->parent;
  }
  std::reverse(path.begin(), path.end());

  // Convert to ROS path message
  global_path.poses.reserve(path.size());
  for (const auto& node : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = node_->now();
    
    double wx, wy;
    costmap_->mapToWorld(node->x, node->y, wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    
    // Set orientation towards next waypoint (or keep goal orientation for last point)
    if (node != path.back()) {
      auto next_node = std::find(path.begin(), path.end(), node);
      ++next_node;
      double next_wx, next_wy;
      costmap_->mapToWorld((*next_node)->x, (*next_node)->y, next_wx, next_wy);
      
      double yaw = std::atan2(next_wy - wy, next_wx - wx);
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      // Use goal orientation for the last point
      pose.pose.orientation = goal.pose.orientation;
    }
    
    global_path.poses.push_back(pose);
  }

  RCLCPP_INFO(node_->get_logger(), "A* found path with %zu waypoints", path.size());
  return global_path;
}

}  // namespace nav2_custom_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner, nav2_core::GlobalPlanner)
