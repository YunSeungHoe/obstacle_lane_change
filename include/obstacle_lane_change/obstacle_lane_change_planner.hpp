#ifndef OBSTACLE_LANE_CHANGE__OBSTACLE_LANE_CHANGE_PLANNER_HPP_
#define OBSTACLE_LANE_CHANGE__OBSTACLE_LANE_CHANGE_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <obstacle_lane_change/obstacle_lane_change_plugin.hpp>
#include <obstacle_lane_change/obstacle_lane_change_handler.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <vector>
#include <map>

#define INITLANELET -1
#define READYLANELET 0
#define INITKEY -1
#define NOLANELET -1
#define INITCOUNT 0
#define FIRSTLANE 1
#define SECONDLANE 2
#define THIRDLANEIDX 2

namespace obstacle_lane_change::lanelet2
{

struct DefaultPlannerParameters
{
  double goal_angle_threshold_deg;
  bool enable_correct_goal_pose;
  bool consider_no_drivable_lanes;
  bool check_footprint_inside_lanes;
};

class DefaultPlanner : public obstacle_lane_change::PlannerPlugin
{
public:
  void initialize(rclcpp::Node * node) override;
  void initialize(rclcpp::Node * node, const HADMapBin::ConstSharedPtr msg) override;
  bool ready() const override;
  LaneletRoute plan(const RoutePoints & points) override;
  MarkerArray visualize(const LaneletRoute & route) const override;
  MarkerArray visualize_debug_footprint(tier4_autoware_utils::LinearRing2d goal_footprint_) const;
  vehicle_info_util::VehicleInfo vehicle_info_;

private:
  using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;
  using Pose = geometry_msgs::msg::Pose;
  bool is_graph_ready_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::ConstLanelets road_lanelets_;
  lanelet::ConstLanelets shoulder_lanelets_;
  obstacle_lane_change_handler::ObstacleLaneChangeHandler route_handler_;

  DefaultPlannerParameters param_;

  rclcpp::Node * node_;
  rclcpp::Subscription<HADMapBin>::SharedPtr map_subscriber_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_goal_footprint_marker_;

  void initialize_common(rclcpp::Node * node);
  void map_callback(const HADMapBin::ConstSharedPtr msg);
  bool check_goal_footprint(
    const lanelet::ConstLanelet & current_lanelet,
    const lanelet::ConstLanelet & combined_prev_lanelet,
    const tier4_autoware_utils::Polygon2d & goal_footprint, double & next_lane_length,
    const double search_margin = 2.0);
  bool is_goal_valid(const geometry_msgs::msg::Pose & goal, lanelet::ConstLanelets path_lanelets);
  Pose refine_goal_height(const Pose & goal, const RouteSections & route_sections);
  void updateRoute(const PlannerPlugin::LaneletRoute & route);
  void clearRoute();
};

} 

#endif  
