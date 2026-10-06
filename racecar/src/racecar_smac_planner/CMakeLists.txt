cmake_minimum_required(VERSION 3.5)
project(racecar_smac_planner)

set(CMAKE_BUILD_TYPE Release) #Debug, Release

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenMP REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
else()
  add_compile_options(-O3 -Wextra -Wdeprecated -fPIC)
endif()

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
  ${OpenMP_INCLUDE_DIRS}
)

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(library_name racecar_smac_planner)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
  angles
  eigen3_cmake_module
)

# Hybrid plugin
add_library(${library_name} SHARED
  src/smac_planner_hybrid.cpp
  src/a_star.cpp
  src/collision_checker.cpp
  src/smoother.cpp
  src/analytic_expansion.cpp
  src/node_hybrid.cpp
  src/node_lattice.cpp
  src/costmap_downsampler.cpp
  src/node_2d.cpp
  src/node_basic.cpp
)

target_link_libraries(${library_name} ${OMPL_LIBRARIES} ${OpenMP_LIBRARIES}  OpenMP::OpenMP_CXX)
target_include_directories(${library_name} PUBLIC ${Eigen3_INCLUDE_DIRS})

ament_target_dependencies(${library_name}
  ${dependencies}
)

pluginlib_export_plugin_description_file(nav2_core smac_plugin_hybrid.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)





ament_export_include_directories(include ${OMPL_INCLUDE_DIRS})
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

add_library(racecar_frontier_kernel SHARED fast/frontier_kernel.cpp)
install(TARGETS racecar_frontier_kernel LIBRARY DESTINATION lib)
find_package(nav2_smac_planner REQUIRED)
add_executable(bench_lazy_distance fast/bench_lazy_distance.cpp)
target_include_directories(bench_lazy_distance PRIVATE include)
ament_target_dependencies(bench_lazy_distance ${dependencies} nav2_smac_planner)
target_link_libraries(bench_lazy_distance ${library_name} ${OMPL_LIBRARIES})
install(TARGETS bench_lazy_distance RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_test(NAME exact_lazy_distance COMMAND bench_lazy_distance)
endif()

find_package(nav2_behavior_tree REQUIRED)
add_library(racecar_goal_preference_bt_node SHARED fast/goal_preference_bt.cpp)
target_compile_definitions(racecar_goal_preference_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(racecar_goal_preference_bt_node nav2_behavior_tree tf2_ros geometry_msgs rclcpp std_msgs nav_msgs nav2_msgs)
install(TARGETS racecar_goal_preference_bt_node LIBRARY DESTINATION lib)
ament_export_dependencies(nav2_behavior_tree)

ament_package()
