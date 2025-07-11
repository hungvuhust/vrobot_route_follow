cmake_minimum_required(VERSION 3.8)
project(vrobot_route_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(mrpt-maps REQUIRED)
find_package(mrpt-math REQUIRED)
find_package(mrpt-gui REQUIRED)
find_package(mrpt-ros2bridge REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(THIS_PACKAGE_DEPENDENCIES
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  vrobot_local_planner
)
foreach(dep ${THIS_PACKAGE_DEPENDENCIES})
  find_package(${dep} REQUIRED)
endforeach()

# Service definition
file(GLOB ${PROJECT_NAME}_interface_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
file(GLOB ${PROJECT_NAME}_srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*)
file(GLOB ${PROJECT_NAME}_action_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*)
rosidl_generate_interfaces(
   ${PROJECT_NAME}
   ${${PROJECT_NAME}_interface_files}
   ${${PROJECT_NAME}_srv_files}
   ${${PROJECT_NAME}_action_files}
   DEPENDENCIES ${THIS_PACKAGE_DEPENDENCIES}
)
ament_export_dependencies(rosidl_default_runtime)

find_package(Drogon CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)




# ========================================================================
# ORGANIZED HEADER STRUCTURE
# ========================================================================

# Core headers
set(CORE_HEADERS
  include/vrobot_route_follow/core/graph_base.hpp
  include/vrobot_route_follow/core/geometric_utils.hpp
  include/vrobot_route_follow/core/pathfinding.hpp
  include/vrobot_route_follow/core/rich_graph.hpp
)

# Algorithm headers
set(ALGORITHM_HEADERS
  include/vrobot_route_follow/algorithms/link_based_planner.hpp
  include/vrobot_route_follow/algorithms/scoring.hpp
)

# Data structure headers
set(DATA_STRUCTURE_HEADERS
  include/vrobot_route_follow/data_structures/node_info.hpp
  include/vrobot_route_follow/data_structures/link_info.hpp
  include/vrobot_route_follow/data_structures/rich_path_result.hpp
  include/vrobot_route_follow/data_structures/path_attributes.hpp
  include/vrobot_route_follow/data_structures/enhanced_path_segment.hpp
)

# Utility headers
set(UTILITY_HEADERS
  include/vrobot_route_follow/utils/nav_conversion.hpp
  include/vrobot_route_follow/utils/visualization.hpp
  include/vrobot_route_follow/utils/database_converter.hpp
  include/vrobot_route_follow/utils/vpath_builder.hpp
)

# Main interface headers
set(INTERFACE_HEADERS
  include/vrobot_route_follow/graph_pose.hpp
)

# All headers (legacy headers removed)
set(ALL_HEADERS
  ${CORE_HEADERS}
  ${ALGORITHM_HEADERS}
  ${DATA_STRUCTURE_HEADERS}
  ${UTILITY_HEADERS}
  ${INTERFACE_HEADERS}
)

# ========================================================================
# SOURCE FILES
# ========================================================================

# Core implementation files
set(CORE_SOURCES
  src/vrobot_route_follow/core/graph_base.cpp
  src/vrobot_route_follow/core/geometric_utils.cpp
  src/vrobot_route_follow/core/pathfinding.cpp
  src/vrobot_route_follow/core/rich_graph.cpp
)

# Algorithm implementation files
set(ALGORITHM_SOURCES
  src/vrobot_route_follow/algorithms/link_based_planner.cpp
  src/vrobot_route_follow/algorithms/scoring.cpp
)

# Data structure implementation files
set(DATA_STRUCTURE_SOURCES
  src/vrobot_route_follow/data_structures/node_info.cpp
  src/vrobot_route_follow/data_structures/link_info.cpp
  src/vrobot_route_follow/data_structures/rich_path_result.cpp
  src/vrobot_route_follow/data_structures/path_attributes.cpp
  src/vrobot_route_follow/data_structures/enhanced_path_segment.cpp
)

# Utility implementation files
set(UTILITY_SOURCES
  src/vrobot_route_follow/utils/nav_conversion.cpp
  src/vrobot_route_follow/utils/visualization.cpp
  src/vrobot_route_follow/utils/database_converter.cpp
  src/vrobot_route_follow/utils/vpath_builder.cpp
)

# Main interface implementation
set(INTERFACE_SOURCES
  src/vrobot_route_follow/graph_pose.cpp
)

# All implementation sources
set(ALL_SOURCES
  ${CORE_SOURCES}
  ${ALGORITHM_SOURCES}
  ${DATA_STRUCTURE_SOURCES}
  ${UTILITY_SOURCES}
  ${INTERFACE_SOURCES}
)

set(READDBR_SRC
  src/test_readdb.cpp
)

aux_source_directory(models MODEL_SRC)

# ========================================================================
# LIBRARY SETUP
# ========================================================================

include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
)


# Create library with separated interface and implementation
add_library(mrpt_graph_pose_lib SHARED
  ${ALL_SOURCES}
  ${ALL_HEADERS}
  ${MODEL_SRC}
)

target_include_directories(mrpt_graph_pose_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/models
)

target_link_libraries(mrpt_graph_pose_lib
    Drogon::Drogon
    mrpt::maps
    mrpt::math
    mrpt::ros2bridge
    mrpt::gui
)

ament_target_dependencies(mrpt_graph_pose_lib
    ${THIS_PACKAGE_DEPENDENCIES}
)
rosidl_target_interfaces(mrpt_graph_pose_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Create header-only library for backward compatibility
add_library(mrpt_graph_pose_headers INTERFACE)
target_include_directories(mrpt_graph_pose_headers
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)



# ========================================================================
# PATH PLANNING SERVICE
# ========================================================================

add_executable(
  path_planning_service
  src/path_planning_service.cpp
  ${MODEL_SRC}
)

target_include_directories(path_planning_service PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/models
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(path_planning_service
  Drogon::Drogon
  mrpt_graph_pose_lib
)

ament_target_dependencies(path_planning_service
  ${THIS_PACKAGE_DEPENDENCIES}
)




# Test client executable
add_executable(
  test_path_planning_client
  src/test_path_planning_client.cpp
)

target_include_directories(test_path_planning_client
  PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_path_planning_client
  mrpt_graph_pose_lib
)

ament_target_dependencies(test_path_planning_client
  ${THIS_PACKAGE_DEPENDENCIES}
)

# ========================================================================
# MOVE TO POSE ACTION SERVER
# ========================================================================

add_executable(
  move_to_pose_action_server
  src/move_to_pose_action_server.cpp
  ${MODEL_SRC}
)

target_include_directories(move_to_pose_action_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/models
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(move_to_pose_action_server
  Drogon::Drogon
  mrpt_graph_pose_lib
)
ament_target_dependencies(move_to_pose_action_server
  ${THIS_PACKAGE_DEPENDENCIES}
)

rosidl_target_interfaces(move_to_pose_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Test move to pose client executable
add_executable(
  test_move_to_pose_client
  src/test_move_to_pose_client.cpp
)

target_include_directories(test_move_to_pose_client
  PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_move_to_pose_client
  mrpt_graph_pose_lib
)

ament_target_dependencies(test_move_to_pose_client
  ${THIS_PACKAGE_DEPENDENCIES}
)

rosidl_target_interfaces(test_move_to_pose_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# ========================================================================
# INSTALLATION
# ========================================================================

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

# Install library
install(
  TARGETS mrpt_graph_pose_lib mrpt_graph_pose_headers
  EXPORT mrpt_graph_pose_libTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install( 
  EXPORT mrpt_graph_pose_libTargets
  FILE mrpt_graph_pose_libTargets.cmake
  NAMESPACE mrpt_graph_pose_lib::
  DESTINATION lib/cmake/mrpt_graph_pose_lib
)

# Install executables
install(
  TARGETS path_planning_service test_path_planning_client move_to_pose_action_server test_move_to_pose_client
  DESTINATION lib/${PROJECT_NAME}
)

# ========================================================================
# PACKAGE CONFIGURATION
# ========================================================================

ament_export_targets(mrpt_graph_pose_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDENCIES})


ament_package()
