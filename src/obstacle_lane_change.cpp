/*
  Autoware obstacle lane change node in carla simulatior, KIAPI and K-city
  map name : Town04, KIAPI, K-city
  node name : obstacle_lane_change_node
  e-mail : yunsh3594@gmail.com 
  made by. Seunghoe Yun
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <obstacle_lane_change/obstacle_lane_change_planner.hpp>
#include <obstacle_lane_change/obstacle_lane_change_handler.hpp>
#include <obstacle_lane_change/obstacle_lane_change_plugin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp> 
#include <autoware_planning_msgs/msg/lane_change_check.hpp>

using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
using AutoPredictedObj = autoware_auto_perception_msgs::msg::PredictedObjects;
using LongIntVec = std::vector<long int>;
using DoubleVec = std::vector<double>;

class ObstacleLaneChange : public rclcpp::Node
{
public:
  ObstacleLaneChange() : Node("obstacle_lane_change")
  {
    currentObjLaneletId = INITLANELET;
    currentLaneletId = INITLANELET;
    lastObjLaneletId = INITLANELET;
    currentObjkey = INITCOUNT;
    currentObjlane = INITCOUNT;
    currentkey = INITCOUNT;
    currentlane = INITCOUNT;
    lastlane = INITCOUNT;
    detectFlag = false;
    lcc_msg_.changeflag = true;
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>( "/localization/pose_estimator/pose", rclcpp::QoS{1}, std::bind(&ObstacleLaneChange::callbackPose, this, std::placeholders::_1));
    sub_vector_map_ = create_subscription<HADMapBin>("/map/vector_map", rclcpp::QoS(1).transient_local(), std::bind(&ObstacleLaneChange::callbackMap, this, std::placeholders::_1));
    object_sub_ = create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>("/perception/object_recognition/objects", rclcpp::QoS(1), std::bind(&ObstacleLaneChange::callbackObject, this,  std::placeholders::_1));
    route_pub_= create_publisher<autoware_planning_msgs::msg::LaneletRoute>("/planning/obstacle_lane_change/route", rclcpp::QoS{1}.transient_local());
    lane_change_check_pub_= create_publisher<autoware_planning_msgs::msg::LaneChangeCheck>("/planning/lane_change_check", rclcpp::QoS{1});

    this->declare_parameter<LongIntVec>("primitives", LongIntVec({}));
    primitives = this->get_parameter("primitives").as_integer_array();
    this->declare_parameter<LongIntVec>("pit_stop_primitives", LongIntVec({}));
    pitStopPrimitives = this->get_parameter("pit_stop_primitives").as_integer_array();
    this->declare_parameter<DoubleVec>("goal_pose.position.x", DoubleVec({}));
    this->declare_parameter<DoubleVec>("goal_pose.position.y", DoubleVec({}));
    this->declare_parameter<DoubleVec>("goal_pose.orientation.z", DoubleVec({}));
    this->declare_parameter<DoubleVec>("goal_pose.orientation.w", DoubleVec({}));
    goalPosition_x = this->get_parameter("goal_pose.position.x").as_double_array();
    goalPosition_y = this->get_parameter("goal_pose.position.y").as_double_array();
    goalOrientation_z = this->get_parameter("goal_pose.orientation.z").as_double_array();
    goalOrientation_w = this->get_parameter("goal_pose.orientation.w").as_double_array();
    this->declare_parameter<int>("lane_number", int());
    laneNum = this->get_parameter("lane_number").as_int();
    primitives2DVector = setPrimitiveVector(primitives, laneNum);
    goalPosition_x2DVec = setPrimitiveVector(goalPosition_x, laneNum);
    goalPosition_y2DVec = setPrimitiveVector(goalPosition_y, laneNum);
  
  }
  autoware_planning_msgs::msg::LaneChangeCheck lcc_msg_;
  lanelet::LaneletMapPtr TR_lanelet_map_ptr_;
  HADMapBin::ConstSharedPtr TR_map_ptr_;
  lanelet::ConstLanelets all_lanelets;
  lanelet::ConstLanelets road_lanelets_;
  AutoPredictedObj::ConstSharedPtr obj_msg_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg_;

  autoware_planning_msgs::msg::LaneletSegment segment;
  autoware_planning_msgs::msg::LaneletSegment emptySegment;
  autoware_planning_msgs::msg::LaneletPrimitive primitive;

  std::vector<DoubleVec> goalPosition_x2DVec;
  std::vector<DoubleVec> goalPosition_y2DVec;
  std::vector<LongIntVec> primitives2DVector;
  std::vector<LongIntVec> roi_lanelets;
  LongIntVec primitives;
  LongIntVec pitStopPrimitives;
  DoubleVec goalPosition_x;
  DoubleVec goalPosition_y;
  DoubleVec goalOrientation_z;
  DoubleVec goalOrientation_w;

  long int currentObjLaneletId;
  long int currentLaneletId;
  long int lastObjLaneletId;
  int laneNum;
  int currentObjkey;
  int currentObjlane;
  int currentkey;
  int currentlane;
  int lastlane;
  int nextObjlane;
  bool detectFlag; 

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr object_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneChangeCheck>::SharedPtr lane_change_check_pub_;

private:
  std::vector<LongIntVec> setPrimitiveVector(const LongIntVec& vec, size_t laneletNum)
  {
    std::vector<LongIntVec> return2DVec;
    return2DVec.reserve((vec.size() + laneletNum - 1) / laneletNum); 
    auto nowVec = vec.begin();
    while (nowVec != vec.end()){
      LongIntVec chunk;
      chunk.reserve(laneletNum);
      auto end = std::next(nowVec, laneletNum);
      std::copy(nowVec, end, std::back_inserter(chunk));
      return2DVec.push_back(std::move(chunk));
      nowVec = end;
    }
    return return2DVec;
  }
  std::vector<DoubleVec> setPrimitiveVector(const DoubleVec& vec, size_t laneletNum)
  {
    std::vector<DoubleVec> return2DVec;
    return2DVec.reserve((vec.size() + laneletNum - 1) / laneletNum); 
    auto nowVec = vec.begin();
    while (nowVec != vec.end()){
      DoubleVec chunk;
      chunk.reserve(laneletNum);
      auto end = std::next(nowVec, laneletNum);
      std::copy(nowVec, end, std::back_inserter(chunk));
      return2DVec.push_back(std::move(chunk));
      nowVec = end;
    }
    return return2DVec;
  }
  std::vector<std::vector<long int>> extractSubMatrix(const std::vector<std::vector<long int>>& matrix, int startRow, int startCol, int endRow, int endCol) 
  {
    std::vector<std::vector<long int>> subMatrix;

    for (int i = startRow; i <= endRow; ++i) {
        std::vector<long int> row;
        for (int j = startCol; j <= endCol; ++j) {
            row.push_back(matrix[i][j]);
        }
        subMatrix.push_back(row);
    }

    return subMatrix;
  }
  void callbackMap(const HADMapBin::ConstSharedPtr msg)
  {
    TR_map_ptr_ = msg;
    TR_lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();

    lanelet::utils::conversion::fromBinMsg(*TR_map_ptr_, TR_lanelet_map_ptr_);
    all_lanelets = lanelet::utils::query::laneletLayer(TR_lanelet_map_ptr_);
    road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
    currentObjLaneletId = READYLANELET;
    currentLaneletId = READYLANELET;
    lastObjLaneletId = READYLANELET;
    RCLCPP_WARN(get_logger(), "Map load Done");
  }
  void callbackPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    pose_msg_ = msg;
  }

  void callbackObject(const AutoPredictedObj::ConstSharedPtr msg)
  {
    obj_msg_ = msg;
    lanelet::ConstLanelets start_lanelets;
    if (lanelet::utils::query::getCurrentLanelets(road_lanelets_, pose_msg_->pose, &start_lanelets))
    {
      for (const auto & st_llt : start_lanelets)
      {
        if(find(pitStopPrimitives.begin(), pitStopPrimitives.end(), st_llt.id()) != pitStopPrimitives.end()) 
        {
          continue;
        }
        currentLaneletId = st_llt.id();
      }      
    }
    if (currentLaneletId == -1) return;
    std::cout << "Get Currentlane id " << currentLaneletId << std::endl;

    // 현재 차량이 위치한 차로 찾기
    currentkey = INITCOUNT;
    for (const auto &lanes : primitives2DVector)
    {
      currentlane = INITCOUNT;
      for (const auto &lane : lanes)
      {
        if (currentLaneletId == lane)
        {
          break;
        }
        currentlane++;
      }
      if (currentLaneletId == lanes[currentlane])
      {
          break;
      }
      currentkey++;
    }
    std::cout << "Get Currentlane Key " << currentkey << std::endl;
    std::cout << "Get Currentlane Lane # " << currentlane << std::endl;
    roi_lanelets = extractSubMatrix(primitives2DVector, currentkey, 0, currentkey+2, laneNum-1);
    std::cout << "ROI primitives split Done ! " << std::endl;

    for (const auto &obj : obj_msg_->objects)
    {
      lanelet::ConstLanelets start_lanelets;
      if (lanelet::utils::query::getCurrentLanelets(road_lanelets_, obj.kinematics.initial_pose_with_covariance.pose, &start_lanelets))
      {
        for (const auto & st_llt : start_lanelets)
        {
          currentObjLaneletId = st_llt.id();
          // SH 장애물이 현재 차량이 있는 laneletid와 +1 laneletid만 탐지
          for (const auto &lane : roi_lanelets)
          {
            for (const auto &inner_lane : lane)
            {
              if (inner_lane == currentObjLaneletId) 
              {
                detectFlag = true;
              }
            }
          }
        }
      }
    }
    // 장애물이 탐지된 경우 한번의 스캔에서 여러개 나온 경우는? 
    // TODO : 다수의 장애물이 한번의 스캔에 있는 경우 => 장애물의 정보를 vector로 생성해서 다수의 장애물의 대해 처리
    // example : 장애물까지의 거리를 예로 들 수 있음 가까운 장애물만 판단?  
    if (detectFlag)
    {
      std::cout << "detection object on lanes" << std::endl; 
      
      if(!lcc_msg_.changeflag) 
      {
        if(currentlane != lastlane) 
        {
          lcc_msg_.changeflag = true;
          lane_change_check_pub_->publish(lcc_msg_);
          std::cout << "currentlane != lastlane (lcc_msg_ : true)" << std::endl;
          return;
        }
        std::cout << "waiting for lane change.. (lcc_msg_ : false)" << std::endl;
        return;
      }
      // 장애물이 전방에 존재하지 않을 수도 있는데? 
      // ego 차량이 위치한 차선에 대한 정보도 필요
      // std::cout << "current obj lanelet id : " << currentObjLaneletId << std::endl;
      lastObjLaneletId = currentObjLaneletId;
      currentObjkey = INITCOUNT;
      for (const auto &lanes : primitives2DVector)
      {
        currentObjlane = INITCOUNT;
        for (const auto &lane : lanes)
        {
          if (currentObjLaneletId == lane)
          {
            break;
          }
          currentObjlane++;
        }
        if (currentObjLaneletId == lanes[currentObjlane])
        {
            break;
        }
        currentObjkey++;
      }
      std::vector<LongIntVec> spilitPrimitives2DVector;
      // carla : currentObjkey+2 / Kiapi : currentObjkey+1
      spilitPrimitives2DVector = extractSubMatrix(primitives2DVector, currentkey, 0, currentObjkey+2, laneNum-1);
      // std::cout << "currentObjkey : " << currentObjkey << std::endl;
      // std::cout << "currentObjlane : " << currentObjlane << std::endl;
      // std::cout << "current lane id  : " << currentObjLaneletId << std::endl;
      // std::cout << "current lane id  : " << primitives2DVector[currentObjkey][currentObjlane] << std::endl;
      // std::cout << "next lanelet id  : " << primitives2DVector[currentObjkey+1][currentObjlane] << std::endl;
      // std::cout << "========================="<< std::endl;

      if(currentlane != currentObjlane) 
      {
        std::cout << "obj lane and ego lane ID is not same" << std::endl;
        return;
      }
      // 차선 변경을 하기 위함
      // 1/3차로 -> 2차로 
      // 2차로 -> 3차로
      switch (currentObjlane)
      {
      case 1:
        nextObjlane = 2;
        break;
      case 0:
      case 2:
        nextObjlane = 1;
        break;
      default:
        break;
      }

      // topic publish
      autoware_planning_msgs::msg::LaneletRoute route_msg_;
      route_msg_.header.stamp = this->get_clock()->now();
      route_msg_.header.frame_id = "map";
      route_msg_.start_pose = pose_msg_->pose;
// key 그대로 idx로 사용
      std::cout << "currentObjkey : " << currentObjkey << std::endl;
      route_msg_.goal_pose.position.x = goalPosition_x2DVec[currentObjkey+1][currentObjlane]; 
      route_msg_.goal_pose.position.y = goalPosition_y2DVec[currentObjkey+1][currentObjlane]; 
      route_msg_.goal_pose.position.z = 0.0;
          
      route_msg_.goal_pose.orientation.x = 0.0;
      route_msg_.goal_pose.orientation.y = 0.0;
      route_msg_.goal_pose.orientation.z = goalOrientation_z[currentObjkey+1];  
      route_msg_.goal_pose.orientation.w = goalOrientation_w[currentObjkey+1];

      for (const auto &out_lane_id : spilitPrimitives2DVector)
      {
        segment = emptySegment;
        segment.preferred_primitive.id = out_lane_id[nextObjlane];
        segment.preferred_primitive.primitive_type = "";
        for (const auto &in_lane_id : out_lane_id)
        {
          if (in_lane_id == NOLANELET) continue;
          primitive.id = in_lane_id;
          primitive.primitive_type = "lane";
          segment.primitives.push_back(primitive);
        }
        route_msg_.segments.push_back(segment);
      }
      route_msg_.uuid.uuid = {209, 239, 15, 91, 197, 87, 68, 179, 62, 19, 3, 36, 111, 114, 35, 231};
      lcc_msg_.changeflag = false;
      lane_change_check_pub_->publish(lcc_msg_);
      route_pub_->publish(route_msg_);
      lastlane = currentlane;
      std::cout << "Lane Change trajectory publish" << std::endl;
    }
    else
    {
      std::cout << "Detection falut !" << std::endl;
    }
  }
};

int main(int argc, char **argv)
{    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleLaneChange>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}