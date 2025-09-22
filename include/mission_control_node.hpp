/********************************************************************************
 * Copyright (C) 2024-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 ********************************************************************************/
#pragma once
#include <chrono>
#include <deque>
#include <optional>

#include "adore_dynamics_adapters.hpp"
#include "adore_dynamics_conversions.hpp"
#include "adore_map/map.hpp"
#include "adore_map/map_loader.hpp"
#include "adore_map/tile_map.hpp"
#include "adore_map_adapters.hpp"
#include "adore_map_conversions.hpp"
#include "adore_ros2_msgs/msg/caution_zone.hpp"
#include "adore_ros2_msgs/msg/goal_point.hpp"
#include "adore_ros2_msgs/msg/map.hpp"
#include "adore_ros2_msgs/msg/route.hpp"
#include "adore_ros2_msgs/msg/vehicle_state_dynamic.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace adore
{
class MissionControlNode : public rclcpp::Node
{
public:

  MissionControlNode( const rclcpp::NodeOptions& options );

private:

  enum GoalType
  {
    PICK_UP,
    DROP_OFF
  };

  struct Goal
  {
    double      x, y;
    std::string label;
    GoalType    type;
  };

  void timer_callback();
  void load_parameters();
  void clicked_point_callback( const geometry_msgs::msg::PointStamped& msg );
  void keep_moving_callback( const adore_ros2_msgs::msg::GoalPoint& msg );
  void vehicle_state_callback( const dynamics::VehicleStateDynamic& msg );
  void create_subscribers();
  void create_publishers();
  void publish_local_map();
  void update_route();
  void publish_caution_zones();

  void reach_goal();

  std::optional<map::Route> current_route = std::nullopt;


  rclcpp::Publisher<RouteAdapter>::SharedPtr                      route_publisher;
  rclcpp::Publisher<MapAdapter>::SharedPtr                        local_map_publisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               goal_reached_publisher;
  rclcpp::Publisher<adore_ros2_msgs::msg::CautionZone>::SharedPtr publisher_caution_zones;


  rclcpp::Subscription<adore_ros2_msgs::msg::GoalPoint>::SharedPtr  keep_moving_subscriber;
  rclcpp::Subscription<StateAdapter>::SharedPtr                     vehicle_state_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_subscriber;
  rclcpp::TimerBase::SharedPtr                                      main_timer;

  std::deque<Goal> goals;
  bool             sent_goal_point = false;

  std::unordered_map<std::string, math::Polygon2d> caution_zones;

  std::optional<dynamics::VehicleStateDynamic> latest_vehicle_state = std::nullopt;
  std::shared_ptr<map::Map>                    road_map             = nullptr;
  std::string                                  map_file_location;


  double local_map_size = 50;
};
} // namespace adore
