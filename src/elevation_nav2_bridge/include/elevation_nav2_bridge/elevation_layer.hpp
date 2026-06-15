#ifndef ELEVATION_NAV2_BRIDGE__ELEVATION_LAYER_HPP_
#define ELEVATION_NAV2_BRIDGE__ELEVATION_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace elevation_nav2_bridge
{

class ElevationLayer : public nav2_costmap_2d::Layer
{
public:
  ElevationLayer() = default;
  ~ElevationLayer() override = default;

  void onInitialize() override;

  void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j) override;

  void reset() override;

  bool isClearable() override;

private:
  void elevationCallback(grid_map_msgs::msg::GridMap::SharedPtr msg);
  bool getLayerIndex(
    const grid_map_msgs::msg::GridMap & map,
    const std::string & layer_name,
    size_t & layer_index);
  bool getElevationAtIndex(
    const grid_map_msgs::msg::GridMap & map,
    size_t layer_index,
    unsigned int x,
    unsigned int y,
    float & elevation);
  unsigned char computeCostFromElevation(float elevation) const;
  bool worldToGridMapIndex(
    const grid_map_msgs::msg::GridMap & map,
    double wx,
    double wy,
    unsigned int & gx,
    unsigned int & gy) const;
  double getGridMapYaw(const grid_map_msgs::msg::GridMap & map) const;
  void getGridMapBounds(
    const grid_map_msgs::msg::GridMap & map,
    double & min_x,
    double & min_y,
    double & max_x,
    double & max_y) const;
  bool getGridMapBoundsInFrame(
    const grid_map_msgs::msg::GridMap & map,
    const std::string & target_frame,
    double & min_x,
    double & min_y,
    double & max_x,
    double & max_y) const;
  std::string normalizeFrameId(const std::string & frame_id) const;
  std::string getGridMapFrame(
    const grid_map_msgs::msg::GridMap & map,
    const std::string & fallback_frame) const;
  bool lookupTransformToFrame(
    const std::string & target_frame,
    const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transform) const;
  bool transformPoint2D(
    const geometry_msgs::msg::TransformStamped & transform,
    double in_x,
    double in_y,
    double & out_x,
    double & out_y) const;
  void publishDebugGrid(const grid_map_msgs::msg::GridMap & map);

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr elevation_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_grid_pub_;
  grid_map_msgs::msg::GridMap::SharedPtr latest_map_;
  std::mutex map_mutex_;

  std::string elevation_topic_{"/elevation_mapping_node/elevation_map"};
  std::string elevation_layer_name_{"elevation"};
  bool unknown_as_obstacle_{false};
  double min_height_{0.0};
  double lethal_height_threshold_{0.25};
  double cost_scale_{252.0};
  bool publish_debug_grid_{true};
  std::string debug_grid_topic_{"/elevation_traversability_debug"};
  double transform_tolerance_{0.2};
  size_t received_map_count_{0};
};

}  // namespace elevation_nav2_bridge

#endif  // ELEVATION_NAV2_BRIDGE__ELEVATION_LAYER_HPP_
