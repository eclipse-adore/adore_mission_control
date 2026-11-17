/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#pragma once
#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "adore_dynamics_adapters.hpp"
#include "adore_dynamics_conversions.hpp"
#include "adore_map/map.hpp"
#include "adore_map/map_loader.hpp"
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
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace adore
{
class MissionControl : public rclcpp::Node
{
public:

  MissionControl( const rclcpp::NodeOptions& options );

private:

  enum GoalType
  {
    CONTINUE,
    STOP
  };

  struct Goal
  {
    double      x, y;
    std::string label;
    GoalType    type;
    bool wait_for_resume = false;    // keep the vehicle waiting until the ride is resumed explicitly.
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
  void update_global_route();
  void publish_caution_zones();
  void drive_back_to_start_callback( const std_msgs::msg::Bool& msg );
  void stop_at_next_virtual_stop_callback( const std_msgs::msg::Bool& msg );
  void move_forward_callback( const std_msgs::msg::Float64& msg );
  void resume_ride_callback( const std_msgs::msg::Bool& msg );

  std::optional<map::Route> route_from_current_state();
  std::optional<Goal>       find_next_virtual_stop();
  void                      insert_stop_goal( const Goal& stop_goal );

  void reach_goal();

  std::optional<map::Route> current_route = std::nullopt;
  std::optional<map::Route> global_route  = std::nullopt;


  rclcpp::Publisher<RouteAdapter>::SharedPtr                      global_route_publisher;
  rclcpp::Publisher<RouteAdapter>::SharedPtr                      local_route_publisher;
  rclcpp::Publisher<MapAdapter>::SharedPtr                        local_map_publisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               goal_reached_publisher;
  rclcpp::Publisher<adore_ros2_msgs::msg::CautionZone>::SharedPtr publisher_caution_zones;


  rclcpp::Subscription<adore_ros2_msgs::msg::GoalPoint>::SharedPtr  keep_moving_subscriber;
  rclcpp::Subscription<StateAdapter>::SharedPtr                     vehicle_state_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_subscriber;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_back_subscriber;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    stop_at_next_virtual_stop_subscriber;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr move_forward_subscriber;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    resume_ride_subscriber;
  rclcpp::TimerBase::SharedPtr                                      main_timer;

  std::deque<Goal> goals;
  std::deque<Goal> all_goals;

  std::unordered_map<std::string, math::Polygon2d> caution_zones;

  std::optional<dynamics::VehicleStateDynamic> latest_vehicle_state = std::nullopt;
  std::shared_ptr<map::Map>                    road_map             = nullptr;
  std::mutex                                   map_mutex_;
  std::string                                  map_file_location;


  double local_map_size = 100;
  std::optional<Goal> start_goal = std::nullopt;

  // A goal counts as reached this far before it
  static constexpr double GOAL_PASS_THRESHOLD = 2.0;

  std::vector<Goal> virtual_stops;
  bool              waiting_at_stop = false;

  double min_stop_distance_vstop       = 15.0;
  double max_stop_lateral_offset_vstop = 10.0;

  // Bounds for the move forward distance
  double min_move_forward_distance = 0.5;
  double max_move_forward_distance = 20.0;
};
} // namespace adore