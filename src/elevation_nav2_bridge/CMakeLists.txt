cmake_minimum_required(VERSION 3.8)
project(elevation_nav2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(elevation_layer SHARED
  src/elevation_layer.cpp
)

target_include_directories(elevation_layer PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

ament_target_dependencies(elevation_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  grid_map_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins/elevation_layer.xml)

install(TARGETS elevation_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins/elevation_layer.xml
  DESTINATION share/${PROJECT_NAME}/plugins
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(elevation_layer)
ament_export_dependencies(
  rclcpp
  nav2_costmap_2d
  pluginlib
  grid_map_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

ament_package()
