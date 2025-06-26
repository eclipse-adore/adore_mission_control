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
#include "mission_control_node.hpp"

#include <type_traits>
using namespace std::chrono_literals;

namespace adore
{

MissionControlNode::MissionControlNode( const rclcpp::NodeOptions& options ) :
  Node( "mission_control", options )
{
  get_first_goal_position();
  road_map = map::MapLoader::load_from_file( map_file_location );

  create_publishers();
  create_subscribers();
}

void
MissionControlNode::create_publishers()
{
  goal_publisher = create_publisher<adore_ros2_msgs::msg::GoalPoint>( "mission/goal_position", 10 );

  route_publisher = create_publisher<adore_ros2_msgs::msg::Route>( "route", 10 );

  local_map_publisher = create_publisher<adore_ros2_msgs::msg::Map>( "local_map", 10 );

  goal_reached_publisher = create_publisher<std_msgs::msg::Bool>( "goal_reached", 10 );
}

void
MissionControlNode::update_route()
{
  if( current_route.has_value() && latest_vehicle_state.has_value() )
  {
    if( current_route->get_length() - current_route->get_s( latest_vehicle_state.value() ) < 0.5 )
    {
      reach_goal();
      current_route = std::nullopt;
    }
  }
  if( !current_route.has_value() && latest_vehicle_state.has_value() && !goals.empty() )
  {
    if( !road_map )
      return;
    current_route = map::Route( latest_vehicle_state.value(), goals.front(), *road_map );

    if( current_route->center_lane.empty() )
      current_route = std::nullopt;
  }
}

void
MissionControlNode::reach_goal()
{
  std_msgs::msg::Bool reached;
  reached.data = true;
  goal_reached_publisher->publish( reached );
  // if( !goals.empty()  )
  //   goals.pop_front();
}

void
MissionControlNode::create_subscribers()
{
  keep_moving_subscriber = create_subscription<adore_ros2_msgs::msg::GoalPoint>( "mission/goal_request", 10,
                                                                                 std::bind( &MissionControlNode::keep_moving_callback, this,
                                                                                            std::placeholders::_1 ) );

  vehicle_state_subscriber = create_subscription<adore_ros2_msgs::msg::VehicleStateDynamic>(
    "vehicle_state/dynamic", 10, std::bind( &MissionControlNode::vehicle_state_callback, this, std::placeholders::_1 ) );

  clicked_point_subscriber = create_subscription<geometry_msgs::msg::PointStamped>( "/clicked_point/goal_position", 10,
                                                                                    std::bind( &MissionControlNode::clicked_point_callback,
                                                                                               this, std::placeholders::_1 ) );

  main_timer = create_wall_timer( 100ms, std::bind( &MissionControlNode::timer_callback, this ) );
}

void
MissionControlNode::get_first_goal_position()
{
  declare_parameter<double>( "local_map_size", 50.0 );
  get_parameter( "local_map_size", local_map_size );
  declare_parameter<double>( "goal_position_x", 0.0 );
  declare_parameter<double>( "goal_position_y", 0.0 );
  declare_parameter( "map file", "" );
  get_parameter( "map file", map_file_location );

  Goal initial_goal;
  initial_goal.label = "goal from launch file";
  get_parameter( "goal_position_x", initial_goal.x );
  get_parameter( "goal_position_y", initial_goal.y );
  goals.push_back( initial_goal );
}

void
MissionControlNode::timer_callback()
{
  sent_goal_point = false;
  publish_goal();
  update_route();

  publish_local_map();
}

void
MissionControlNode::publish_goal() // TODO remove this once no more nodes
                                   // rely on it
{
  if( goal_publisher->get_subscription_count() > 0 )
  {
    adore_ros2_msgs::msg::GoalPoint goal_point;
    goal_point.x_position      = goals.front().x;
    goal_point.y_position      = goals.front().y;
    goal_point.header.frame_id = "world";
    goal_publisher->publish( goal_point );
    sent_goal_point = true;
  }
}

void
MissionControlNode::publish_local_map()
{
  if( !road_map.has_value() || !latest_vehicle_state.has_value() )
    return;
  auto local_map = road_map->get_submap( latest_vehicle_state.value(), local_map_size, local_map_size );
  local_map_publisher->publish( map::conversions::to_ros_msg( local_map ) );

  if( current_route.has_value() )
  {
    auto local_route = current_route;
    local_route->map = std::make_shared<map::Map>( local_map );
    route_publisher->publish( map::conversions::to_ros_msg( *local_route ) );
  }
  else
  {
    // send empty route anyway
    adore_ros2_msgs::msg::Route empty;
    route_publisher->publish( empty );
  }
}

void
MissionControlNode::keep_moving_callback( const adore_ros2_msgs::msg::GoalPoint& msg )
{
  Goal keep_moving_goal;
  keep_moving_goal.label = "keep moving goal";
  keep_moving_goal.x     = msg.x_position;
  keep_moving_goal.y     = msg.y_position;
  sent_goal_point        = false;
  goals.pop_front();
  goals.push_front( keep_moving_goal );
  current_route = std::nullopt;
}

void
MissionControlNode::clicked_point_callback( const geometry_msgs::msg::PointStamped& msg )
{
  Goal keep_moving_goal;
  keep_moving_goal.label = "custom set goal";
  keep_moving_goal.x     = msg.point.x;
  keep_moving_goal.y     = msg.point.y;
  sent_goal_point        = false;
  goals.push_front( keep_moving_goal );
}

void
MissionControlNode::vehicle_state_callback( const adore_ros2_msgs::msg::VehicleStateDynamic& msg )
{
  latest_vehicle_state = dynamics::conversions::to_cpp_type( msg );
}

} // namespace adore

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  auto node = std::make_shared<adore::MissionControlNode>( rclcpp::NodeOptions{} );
  rclcpp::spin( node );
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( adore::MissionControlNode )
