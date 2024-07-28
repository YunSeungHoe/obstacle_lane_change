#ifndef OBSTACLE_LANE_CHANGE__OBSTACLE_LANE_CHANGE_PLUGIN_HPP_
#define OBSTACLE_LANE_CHANGE__OBSTACLE_LANE_CHANGE_PLUGIN_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>

#include <vector>

namespace obstacle_lane_change
{

class PlannerPlugin
{
public:
  using RoutePoints = std::vector<geometry_msgs::msg::Pose>;
  using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  virtual ~PlannerPlugin() = default;
  virtual void initialize(rclcpp::Node * node) = 0;
  virtual void initialize(rclcpp::Node * node, const HADMapBin::ConstSharedPtr msg) = 0;
  virtual bool ready() const = 0;
  virtual LaneletRoute plan(const RoutePoints & points) = 0;
  virtual MarkerArray visualize(const LaneletRoute & route) const = 0;
  virtual void updateRoute(const LaneletRoute & route) = 0;
  virtual void clearRoute() = 0;
};

}  // namespace obstacle_lane_change

#endif  
