cmake_minimum_required(VERSION 3.8)

get_filename_component(CURRENT_DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(TOLOWER ${CURRENT_DIR_NAME} PROJECT)
project(${PROJECT})

include($ENV{SOURCE_DIRECTORY}/libraries/adore_libraries.cmake)

include(requirements.cmake)
set(CMAKE_CXX_STANDARD 20)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)

# Ensure the Release build uses -O3 for optimization
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(${PROJECT} 
  src/mission_control_node.cpp
)

target_include_directories(${PROJECT} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_all_target_include_directories(${PROJECT})
add_all_target_link_libraries(${PROJECT})

install(TARGETS
    ${PROJECT}
  DESTINATION lib/${PROJECT}
)

add_library(missioncontrolnode_component SHARED src/mission_control_node.cpp)
add_all_target_include_directories(missioncontrolnode_component)
add_all_target_link_libraries(missioncontrolnode_component)
rclcpp_components_register_node(
  missioncontrolnode_component
  PLUGIN "adore::MissionControlNode"
  EXECUTABLE missioncontrolnode
)
install(TARGETS missioncontrolnode_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
