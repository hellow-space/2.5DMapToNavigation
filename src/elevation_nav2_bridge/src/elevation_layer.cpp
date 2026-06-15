#include "elevation_nav2_bridge/elevation_layer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace elevation_nav2_bridge
{

void ElevationLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("ElevationLayer failed to lock lifecycle node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter(
    "elevation_topic",
    rclcpp::ParameterValue(std::string("/elevation_mapping_node/elevation_map")));
  declareParameter("cost_source", rclcpp::ParameterValue(std::string("elevation")));
  declareParameter("elevation_layer_name", rclcpp::ParameterValue(std::string("elevation")));
  declareParameter(
    "traversability_layer_name",
    rclcpp::ParameterValue(std::string("traversability")));
  declareParameter("unknown_as_obstacle", rclcpp::ParameterValue(false));
  declareParameter("min_height", rclcpp::ParameterValue(0.0));
  declareParameter("lethal_height_threshold", rclcpp::ParameterValue(0.25));
  declareParameter("cost_scale", rclcpp::ParameterValue(252.0));
  declareParameter("free_traversability_threshold", rclcpp::ParameterValue(0.8));
  declareParameter("lethal_traversability_threshold", rclcpp::ParameterValue(0.25));
  declareParameter("traversability_cost_scale", rclcpp::ParameterValue(252.0));
  declareParameter("enable_step_height_check", rclcpp::ParameterValue(true));
  declareParameter("max_step_height", rclcpp::ParameterValue(0.15));
  declareParameter("comfortable_step_height", rclcpp::ParameterValue(0.06));
  declareParameter("max_drop_height", rclcpp::ParameterValue(0.12));
  declareParameter("step_neighbor_radius", rclcpp::ParameterValue(1));
  declareParameter("publish_debug_grid", rclcpp::ParameterValue(true));
  declareParameter(
    "debug_grid_topic",
    rclcpp::ParameterValue(std::string("/elevation_traversability_debug")));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.2));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".elevation_topic", elevation_topic_);
  node->get_parameter(name_ + ".cost_source", cost_source_name_);
  node->get_parameter(name_ + ".elevation_layer_name", elevation_layer_name_);
  node->get_parameter(name_ + ".traversability_layer_name", traversability_layer_name_);
  node->get_parameter(name_ + ".unknown_as_obstacle", unknown_as_obstacle_);
  node->get_parameter(name_ + ".min_height", min_height_);
  node->get_parameter(name_ + ".lethal_height_threshold", lethal_height_threshold_);
  node->get_parameter(name_ + ".cost_scale", cost_scale_);
  node->get_parameter(name_ + ".free_traversability_threshold", free_traversability_threshold_);
  node->get_parameter(name_ + ".lethal_traversability_threshold", lethal_traversability_threshold_);
  node->get_parameter(name_ + ".traversability_cost_scale", traversability_cost_scale_);
  node->get_parameter(name_ + ".enable_step_height_check", enable_step_height_check_);
  node->get_parameter(name_ + ".max_step_height", max_step_height_);
  node->get_parameter(name_ + ".comfortable_step_height", comfortable_step_height_);
  node->get_parameter(name_ + ".max_drop_height", max_drop_height_);
  int step_neighbor_radius = 1;
  node->get_parameter(name_ + ".step_neighbor_radius", step_neighbor_radius);
  node->get_parameter(name_ + ".publish_debug_grid", publish_debug_grid_);
  node->get_parameter(name_ + ".debug_grid_topic", debug_grid_topic_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);

  cost_source_ = parseCostSource(cost_source_name_);
  if (cost_source_ == CostSource::Elevation) {
    cost_source_name_ = "elevation";
  } else if (cost_source_ == CostSource::Traversability) {
    cost_source_name_ = "traversability";
  } else {
    cost_source_name_ = "fused";
  }
  free_traversability_threshold_ = std::min(
    std::max(free_traversability_threshold_, 0.0), 1.0);
  lethal_traversability_threshold_ = std::min(
    std::max(lethal_traversability_threshold_, 0.0), 1.0);
  if (free_traversability_threshold_ <= lethal_traversability_threshold_) {
    RCLCPP_WARN(
      logger_,
      "ElevationLayer '%s' got invalid traversability thresholds free=%.3f lethal=%.3f; "
      "using free=0.800 lethal=0.250.",
      name_.c_str(),
      free_traversability_threshold_,
      lethal_traversability_threshold_);
    free_traversability_threshold_ = 0.8;
    lethal_traversability_threshold_ = 0.25;
  }
  if (max_step_height_ <= 0.0) {
    RCLCPP_WARN(
      logger_,
      "ElevationLayer '%s' got non-positive max_step_height %.3f; using 0.150.",
      name_.c_str(),
      max_step_height_);
    max_step_height_ = 0.15;
  }
  if (max_drop_height_ <= 0.0) {
    RCLCPP_WARN(
      logger_,
      "ElevationLayer '%s' got non-positive max_drop_height %.3f; using max_step_height %.3f.",
      name_.c_str(),
      max_drop_height_,
      max_step_height_);
    max_drop_height_ = max_step_height_;
  }
  comfortable_step_height_ = std::max(comfortable_step_height_, 0.0);
  if (comfortable_step_height_ >= max_step_height_) {
    RCLCPP_WARN(
      logger_,
      "ElevationLayer '%s' got comfortable_step_height %.3f >= max_step_height %.3f; "
      "using half of max_step_height.",
      name_.c_str(),
      comfortable_step_height_,
      max_step_height_);
    comfortable_step_height_ = 0.5 * max_step_height_;
  }
  if (step_neighbor_radius < 1) {
    RCLCPP_WARN(
      logger_,
      "ElevationLayer '%s' got step_neighbor_radius %d; using 1.",
      name_.c_str(),
      step_neighbor_radius);
    step_neighbor_radius = 1;
  }
  step_neighbor_radius_ = static_cast<unsigned int>(step_neighbor_radius);
  if (transform_tolerance_ < 0.0) {
    RCLCPP_WARN(
      logger_,
      "ElevationLayer '%s' got negative transform_tolerance %.3f; using 0.0.",
      name_.c_str(),
      transform_tolerance_);
    transform_tolerance_ = 0.0;
  }

  elevation_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    elevation_topic_,
    rclcpp::QoS(10),
    std::bind(&ElevationLayer::elevationCallback, this, std::placeholders::_1));

  if (publish_debug_grid_) {
    debug_grid_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      debug_grid_topic_,
      rclcpp::QoS(1));
  }

  current_ = true;

  RCLCPP_INFO(
    logger_,
    "Initialized ElevationLayer '%s' enabled=%s elevation_topic='%s' cost_source='%s' "
    "elevation_layer='%s' traversability_layer='%s' min_height=%.3f "
    "lethal_height_threshold=%.3f cost_scale=%.3f trav_free=%.3f trav_lethal=%.3f "
    "max_step=%.3f max_drop=%.3f step_radius=%u transform_tolerance=%.3f "
    "debug_grid=%s '%s'",
    name_.c_str(),
    enabled_ ? "true" : "false",
    elevation_topic_.c_str(),
    cost_source_name_.c_str(),
    elevation_layer_name_.c_str(),
    traversability_layer_name_.c_str(),
    min_height_,
    lethal_height_threshold_,
    cost_scale_,
    free_traversability_threshold_,
    lethal_traversability_threshold_,
    max_step_height_,
    max_drop_height_,
    step_neighbor_radius_,
    transform_tolerance_,
    publish_debug_grid_ ? "true" : "false",
    debug_grid_topic_.c_str());
}

void ElevationLayer::elevationCallback(grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  const auto layer_count = msg->layers.size();
  const auto frame_id = msg->header.frame_id;
  size_t received_map_count = 0;
  grid_map_msgs::msg::GridMap::SharedPtr debug_map;

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = std::move(msg);
    debug_map = latest_map_;
    ++received_map_count_;
    received_map_count = received_map_count_;
  }

  current_ = true;

  RCLCPP_DEBUG_THROTTLE(
    logger_,
    *clock_,
    5000,
    "ElevationLayer '%s' received GridMap #%zu from '%s' frame='%s' layers=%zu. "
    "Map cached for elevation cost conversion.",
    name_.c_str(),
    received_map_count,
    elevation_topic_.c_str(),
    frame_id.c_str(),
    layer_count);

  if (publish_debug_grid_ && debug_map) {
    publishDebugGrid(*debug_map);
  }
}

bool ElevationLayer::getLayerIndex(
  const grid_map_msgs::msg::GridMap & map,
  const std::string & layer_name,
  size_t & layer_index)
{
  const auto it = std::find(map.layers.begin(), map.layers.end(), layer_name);
  if (it == map.layers.end()) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      5000,
      "ElevationLayer '%s' did not find GridMap layer '%s'. Available layers=%zu.",
      name_.c_str(),
      layer_name.c_str(),
      map.layers.size());
    return false;
  }

  layer_index = static_cast<size_t>(std::distance(map.layers.begin(), it));
  RCLCPP_DEBUG_THROTTLE(
    logger_,
    *clock_,
    5000,
    "ElevationLayer '%s' using GridMap layer '%s' at index %zu.",
    name_.c_str(),
    layer_name.c_str(),
    layer_index);
  return true;
}

bool ElevationLayer::getLayerValueAtIndex(
  const grid_map_msgs::msg::GridMap & map,
  size_t layer_index,
  unsigned int x,
  unsigned int y,
  float & value) const
{
  if (layer_index >= map.data.size()) {
    return false;
  }

  const auto & layer = map.data[layer_index];
  if (layer.data.empty() || layer.layout.dim.size() < 2) {
    return false;
  }

  const auto & dim0 = layer.layout.dim[0];
  const auto & dim1 = layer.layout.dim[1];
  if (dim0.size == 0 || dim1.size == 0) {
    return false;
  }

  size_t data_index = 0;
  if (dim0.label == "column_index" && dim1.label == "row_index") {
    const size_t cols = dim0.size;
    const size_t rows = dim1.size;
    if (x >= cols || y >= rows) {
      return false;
    }
    data_index = static_cast<size_t>(x) * rows + static_cast<size_t>(y);
  } else if (dim0.label == "row_index" && dim1.label == "column_index") {
    const size_t rows = dim0.size;
    const size_t cols = dim1.size;
    if (x >= cols || y >= rows) {
      return false;
    }
    data_index = static_cast<size_t>(y) * cols + static_cast<size_t>(x);
  } else {
    const size_t cols = dim0.size;
    const size_t rows = dim1.size;
    if (x >= cols || y >= rows) {
      return false;
    }
    data_index = static_cast<size_t>(x) * rows + static_cast<size_t>(y);
  }

  if (data_index >= layer.data.size()) {
    return false;
  }

  value = layer.data[data_index];
  return std::isfinite(value);
}

ElevationLayer::CostSource ElevationLayer::parseCostSource(const std::string & source) const
{
  if (source == "elevation") {
    return CostSource::Elevation;
  }
  if (source == "traversability") {
    return CostSource::Traversability;
  }
  if (source == "fused") {
    return CostSource::Fused;
  }

  RCLCPP_WARN(
    logger_,
    "ElevationLayer '%s' got unknown cost_source '%s'; using 'elevation'.",
    name_.c_str(),
    source.c_str());
  return CostSource::Elevation;
}

bool ElevationLayer::needsElevationLayer() const
{
  return cost_source_ == CostSource::Elevation || cost_source_ == CostSource::Fused;
}

bool ElevationLayer::needsTraversabilityLayer() const
{
  return cost_source_ == CostSource::Traversability || cost_source_ == CostSource::Fused;
}

unsigned char ElevationLayer::computeCostFromElevation(float elevation) const
{
  if (!std::isfinite(elevation)) {
    return unknown_as_obstacle_ ?
           nav2_costmap_2d::NO_INFORMATION :
           nav2_costmap_2d::FREE_SPACE;
  }

  if (elevation >= lethal_height_threshold_) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  if (elevation <= min_height_) {
    return nav2_costmap_2d::FREE_SPACE;
  }

  const double height_range = lethal_height_threshold_ - min_height_;
  if (height_range <= std::numeric_limits<double>::epsilon()) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  const double normalized = (static_cast<double>(elevation) - min_height_) / height_range;
  const double scaled_cost = std::min(std::max(normalized * cost_scale_, 0.0), 252.0);
  return static_cast<unsigned char>(std::lround(scaled_cost));
}

unsigned char ElevationLayer::computeCostFromTraversability(float traversability) const
{
  if (!std::isfinite(traversability)) {
    return unknown_as_obstacle_ ?
           nav2_costmap_2d::NO_INFORMATION :
           nav2_costmap_2d::FREE_SPACE;
  }

  const double clipped = std::min(std::max(static_cast<double>(traversability), 0.0), 1.0);
  if (clipped <= lethal_traversability_threshold_) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  if (clipped >= free_traversability_threshold_) {
    return nav2_costmap_2d::FREE_SPACE;
  }

  const double range = free_traversability_threshold_ - lethal_traversability_threshold_;
  if (range <= std::numeric_limits<double>::epsilon()) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  const double normalized_untraversable =
    (free_traversability_threshold_ - clipped) / range;
  const double scaled_cost = std::min(
    std::max(normalized_untraversable * traversability_cost_scale_, 1.0), 252.0);
  return static_cast<unsigned char>(std::lround(scaled_cost));
}

unsigned char ElevationLayer::computeCostFromStep(
  double max_step_up,
  double max_step_down) const
{
  const auto ratio = [this](double value, double limit) {
      if (value <= comfortable_step_height_) {
        return 0.0;
      }
      const double range = limit - comfortable_step_height_;
      if (range <= std::numeric_limits<double>::epsilon()) {
        return 1.0;
      }
      return std::min(std::max((value - comfortable_step_height_) / range, 0.0), 1.0);
    };

  const double up_ratio = ratio(max_step_up, max_step_height_);
  const double down_ratio = ratio(max_step_down, max_drop_height_);
  const double step_cost = std::max(up_ratio, down_ratio) * 252.0;
  if (step_cost <= 0.0) {
    return nav2_costmap_2d::FREE_SPACE;
  }
  return static_cast<unsigned char>(std::lround(std::min(std::max(step_cost, 1.0), 252.0)));
}

bool ElevationLayer::getStepHeightMetricsAtIndex(
  const grid_map_msgs::msg::GridMap & map,
  size_t elevation_layer_index,
  unsigned int gx,
  unsigned int gy,
  float center_elevation,
  double & max_step_up,
  double & max_step_down) const
{
  max_step_up = 0.0;
  max_step_down = 0.0;
  bool found_neighbor = false;

  const auto center_x = static_cast<int>(gx);
  const auto center_y = static_cast<int>(gy);
  const auto radius = static_cast<int>(step_neighbor_radius_);

  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }

      const int nx = center_x + dx;
      const int ny = center_y + dy;
      if (nx < 0 || ny < 0) {
        continue;
      }

      float neighbor_elevation = 0.0F;
      if (!getLayerValueAtIndex(
          map,
          elevation_layer_index,
          static_cast<unsigned int>(nx),
          static_cast<unsigned int>(ny),
          neighbor_elevation))
      {
        continue;
      }

      found_neighbor = true;
      const double delta =
        static_cast<double>(neighbor_elevation) - static_cast<double>(center_elevation);
      max_step_up = std::max(max_step_up, delta);
      max_step_down = std::max(max_step_down, -delta);
    }
  }

  return found_neighbor;
}

bool ElevationLayer::computeCostAtGridMapIndex(
  const grid_map_msgs::msg::GridMap & map,
  size_t elevation_layer_index,
  size_t traversability_layer_index,
  unsigned int gx,
  unsigned int gy,
  unsigned char & cost,
  bool & step_limited) const
{
  step_limited = false;

  if (cost_source_ == CostSource::Elevation) {
    float elevation = 0.0F;
    if (!getLayerValueAtIndex(map, elevation_layer_index, gx, gy, elevation)) {
      return false;
    }
    cost = computeCostFromElevation(elevation);
    return true;
  }

  float traversability = 0.0F;
  if (!getLayerValueAtIndex(map, traversability_layer_index, gx, gy, traversability)) {
    return false;
  }

  const auto traversability_cost = computeCostFromTraversability(traversability);
  if (cost_source_ == CostSource::Traversability) {
    cost = traversability_cost;
    return true;
  }

  float elevation = 0.0F;
  if (!getLayerValueAtIndex(map, elevation_layer_index, gx, gy, elevation)) {
    return false;
  }

  unsigned char step_cost = nav2_costmap_2d::FREE_SPACE;
  if (enable_step_height_check_) {
    double max_step_up = 0.0;
    double max_step_down = 0.0;
    if (getStepHeightMetricsAtIndex(
        map,
        elevation_layer_index,
        gx,
        gy,
        elevation,
        max_step_up,
        max_step_down))
    {
      if (max_step_up > max_step_height_ || max_step_down > max_drop_height_) {
        cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        step_limited = true;
        return true;
      }
      step_cost = computeCostFromStep(max_step_up, max_step_down);
    }
  }

  cost = std::max(traversability_cost, step_cost);
  return true;
}

bool ElevationLayer::worldToGridMapIndex(
  const grid_map_msgs::msg::GridMap & map,
  double wx,
  double wy,
  unsigned int & gx,
  unsigned int & gy) const
{
  const double resolution = map.info.resolution;
  const double length_x = map.info.length_x;
  const double length_y = map.info.length_y;
  if (resolution <= 0.0 || length_x <= 0.0 || length_y <= 0.0) {
    return false;
  }

  const double center_x = map.info.pose.position.x;
  const double center_y = map.info.pose.position.y;
  const double yaw = getGridMapYaw(map);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double dx = wx - center_x;
  const double dy = wy - center_y;

  // Transform world coordinates into the GridMap local frame. The GridMap
  // message stores data in grid_map convention: row index grows toward -X and
  // column index grows toward -Y from the map center.
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;
  const double row_coord = (length_x / 2.0 - local_x) / resolution;
  const double col_coord = (length_y / 2.0 - local_y) / resolution;

  if (row_coord < 0.0 || col_coord < 0.0) {
    return false;
  }

  const auto rows = static_cast<unsigned int>(std::floor(length_x / resolution));
  const auto cols = static_cast<unsigned int>(std::floor(length_y / resolution));
  if (rows == 0 || cols == 0) {
    return false;
  }

  const auto row = static_cast<unsigned int>(std::floor(row_coord));
  const auto col = static_cast<unsigned int>(std::floor(col_coord));
  if (row >= rows || col >= cols) {
    return false;
  }

  gx = col;
  gy = row;

  return true;
}

double ElevationLayer::getGridMapYaw(const grid_map_msgs::msg::GridMap & map) const
{
  const auto & q = map.info.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void ElevationLayer::getGridMapBounds(
  const grid_map_msgs::msg::GridMap & map,
  double & min_x,
  double & min_y,
  double & max_x,
  double & max_y) const
{
  const double half_x = map.info.length_x / 2.0;
  const double half_y = map.info.length_y / 2.0;
  const double center_x = map.info.pose.position.x;
  const double center_y = map.info.pose.position.y;
  const double yaw = getGridMapYaw(map);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  min_x = std::numeric_limits<double>::max();
  min_y = std::numeric_limits<double>::max();
  max_x = std::numeric_limits<double>::lowest();
  max_y = std::numeric_limits<double>::lowest();

  const double corners[4][2] = {
    {half_x, half_y},
    {half_x, -half_y},
    {-half_x, half_y},
    {-half_x, -half_y},
  };

  for (const auto & corner : corners) {
    const double wx = center_x + cos_yaw * corner[0] - sin_yaw * corner[1];
    const double wy = center_y + sin_yaw * corner[0] + cos_yaw * corner[1];
    min_x = std::min(min_x, wx);
    min_y = std::min(min_y, wy);
    max_x = std::max(max_x, wx);
    max_y = std::max(max_y, wy);
  }
}

bool ElevationLayer::getGridMapBoundsInFrame(
  const grid_map_msgs::msg::GridMap & map,
  const std::string & target_frame,
  double & min_x,
  double & min_y,
  double & max_x,
  double & max_y) const
{
  const double resolution = map.info.resolution;
  if (resolution <= 0.0 || map.info.length_x <= 0.0 || map.info.length_y <= 0.0) {
    return false;
  }

  const std::string normalized_target = normalizeFrameId(target_frame);
  const std::string gridmap_frame = getGridMapFrame(map, normalized_target);
  geometry_msgs::msg::TransformStamped gridmap_to_target;
  if (!lookupTransformToFrame(normalized_target, gridmap_frame, gridmap_to_target)) {
    return false;
  }

  const double half_x = map.info.length_x / 2.0;
  const double half_y = map.info.length_y / 2.0;
  const double center_x = map.info.pose.position.x;
  const double center_y = map.info.pose.position.y;
  const double yaw = getGridMapYaw(map);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  min_x = std::numeric_limits<double>::max();
  min_y = std::numeric_limits<double>::max();
  max_x = std::numeric_limits<double>::lowest();
  max_y = std::numeric_limits<double>::lowest();

  const double corners[4][2] = {
    {half_x, half_y},
    {half_x, -half_y},
    {-half_x, half_y},
    {-half_x, -half_y},
  };

  for (const auto & corner : corners) {
    const double wx = center_x + cos_yaw * corner[0] - sin_yaw * corner[1];
    const double wy = center_y + sin_yaw * corner[0] + cos_yaw * corner[1];
    double tx = 0.0;
    double ty = 0.0;
    if (!transformPoint2D(gridmap_to_target, wx, wy, tx, ty)) {
      return false;
    }
    min_x = std::min(min_x, tx);
    min_y = std::min(min_y, ty);
    max_x = std::max(max_x, tx);
    max_y = std::max(max_y, ty);
  }

  return true;
}

std::string ElevationLayer::normalizeFrameId(const std::string & frame_id) const
{
  auto normalized = frame_id;
  while (!normalized.empty() && normalized.front() == '/') {
    normalized.erase(normalized.begin());
  }
  return normalized;
}

std::string ElevationLayer::getGridMapFrame(
  const grid_map_msgs::msg::GridMap & map,
  const std::string & fallback_frame) const
{
  const auto gridmap_frame = normalizeFrameId(map.header.frame_id);
  if (!gridmap_frame.empty()) {
    return gridmap_frame;
  }

  RCLCPP_WARN_THROTTLE(
    logger_,
    *clock_,
    5000,
    "ElevationLayer '%s' received GridMap with an empty header.frame_id; "
    "falling back to costmap frame '%s'.",
    name_.c_str(),
    fallback_frame.c_str());
  return fallback_frame;
}

bool ElevationLayer::lookupTransformToFrame(
  const std::string & target_frame,
  const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transform) const
{
  const auto normalized_target = normalizeFrameId(target_frame);
  const auto normalized_source = normalizeFrameId(source_frame);
  if (normalized_target.empty() || normalized_source.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      5000,
      "ElevationLayer '%s' cannot lookup transform because target='%s' source='%s'.",
      name_.c_str(),
      normalized_target.c_str(),
      normalized_source.c_str());
    return false;
  }

  if (normalized_target == normalized_source) {
    transform = geometry_msgs::msg::TransformStamped();
    transform.header.frame_id = normalized_target;
    transform.child_frame_id = normalized_source;
    transform.transform.rotation.w = 1.0;
    return true;
  }

  if (tf_ == nullptr) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      5000,
      "ElevationLayer '%s' cannot transform from '%s' to '%s' because tf buffer is null.",
      name_.c_str(),
      normalized_source.c_str(),
      normalized_target.c_str());
    return false;
  }

  try {
    transform = tf_->lookupTransform(
      normalized_target,
      normalized_source,
      tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      5000,
      "ElevationLayer '%s' failed to transform from '%s' to '%s': %s",
      name_.c_str(),
      normalized_source.c_str(),
      normalized_target.c_str(),
      ex.what());
    return false;
  }
}

bool ElevationLayer::transformPoint2D(
  const geometry_msgs::msg::TransformStamped & transform,
  double in_x,
  double in_y,
  double & out_x,
  double & out_y) const
{
  const auto & translation = transform.transform.translation;
  const auto & q = transform.transform.rotation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;

  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;
  const double xy = x * y;
  const double wz = w * z;

  const double m00 = 1.0 - 2.0 * (yy + zz);
  const double m01 = 2.0 * (xy - wz);
  const double m10 = 2.0 * (xy + wz);
  const double m11 = 1.0 - 2.0 * (xx + zz);

  out_x = translation.x + m00 * in_x + m01 * in_y;
  out_y = translation.y + m10 * in_x + m11 * in_y;
  return std::isfinite(out_x) && std::isfinite(out_y);
}


void ElevationLayer::publishDebugGrid(const grid_map_msgs::msg::GridMap & map)
{
  if (!debug_grid_pub_) {
    return;
  }

  const double resolution = map.info.resolution;
  if (resolution <= 0.0 || map.info.length_x <= 0.0 || map.info.length_y <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      5000,
      "ElevationLayer '%s' cannot publish debug grid because GridMap geometry is invalid.",
      name_.c_str());
    return;
  }

  size_t elevation_layer_index = 0;
  size_t traversability_layer_index = 0;
  if (needsElevationLayer() && !getLayerIndex(map, elevation_layer_name_, elevation_layer_index)) {
    return;
  }
  if (needsTraversabilityLayer() &&
    !getLayerIndex(map, traversability_layer_name_, traversability_layer_index))
  {
    return;
  }

  double min_x = 0.0;
  double min_y = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
  getGridMapBounds(map, min_x, min_y, max_x, max_y);

  const auto width = static_cast<unsigned int>(std::ceil((max_x - min_x) / resolution));
  const auto height = static_cast<unsigned int>(std::ceil((max_y - min_y) / resolution));
  if (width == 0 || height == 0) {
    return;
  }

  nav_msgs::msg::OccupancyGrid debug_grid;
  debug_grid.header = map.header;
  debug_grid.info.resolution = static_cast<float>(resolution);
  debug_grid.info.width = width;
  debug_grid.info.height = height;
  debug_grid.info.origin.position.x = min_x;
  debug_grid.info.origin.position.y = min_y;
  debug_grid.info.origin.position.z = 0.0;
  debug_grid.info.origin.orientation.w = 1.0;
  debug_grid.data.assign(static_cast<size_t>(width) * static_cast<size_t>(height), -1);

  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const size_t out_index = static_cast<size_t>(y) * width + static_cast<size_t>(x);
      const double wx = min_x + (static_cast<double>(x) + 0.5) * resolution;
      const double wy = min_y + (static_cast<double>(y) + 0.5) * resolution;
      unsigned int gx = 0;
      unsigned int gy = 0;
      unsigned char cost = nav2_costmap_2d::FREE_SPACE;
      bool step_limited = false;
      if (!worldToGridMapIndex(map, wx, wy, gx, gy) || !computeCostAtGridMapIndex(
          map,
          elevation_layer_index,
          traversability_layer_index,
          gx,
          gy,
          cost,
          step_limited))
      {
        debug_grid.data[out_index] = -1;
        continue;
      }

      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        debug_grid.data[out_index] = -1;
      } else if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        debug_grid.data[out_index] = 100;
      } else if (cost == nav2_costmap_2d::FREE_SPACE) {
        debug_grid.data[out_index] = 0;
      } else {
        const int occupancy_cost = static_cast<int>(
          std::lround(static_cast<double>(cost) * 99.0 / 252.0));
        debug_grid.data[out_index] = static_cast<int8_t>(
          std::min(std::max(occupancy_cost, 1), 99));
      }
    }
  }

  debug_grid_pub_->publish(debug_grid);
}

void ElevationLayer::updateBounds(
  double robot_x,
  double robot_y,
  double robot_yaw,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  (void) robot_x;
  (void) robot_y;
  (void) robot_yaw;

  if (!enabled_) {
    return;
  }

  grid_map_msgs::msg::GridMap::SharedPtr map;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = latest_map_;
  }

  if (!map) {
    return;
  }

  const double resolution = map->info.resolution;
  if (resolution <= 0.0 || map->info.length_x <= 0.0 || map->info.length_y <= 0.0) {
    current_ = false;
    return;
  }

  double map_min_x = 0.0;
  double map_min_y = 0.0;
  double map_max_x = 0.0;
  double map_max_y = 0.0;
  const std::string costmap_frame = normalizeFrameId(layered_costmap_->getGlobalFrameID());
  if (!getGridMapBoundsInFrame(*map, costmap_frame, map_min_x, map_min_y, map_max_x, map_max_y)) {
    current_ = false;
    return;
  }

  // Request costmap updates over the full latest elevation map footprint.
  // The GridMap may be in a different frame than the costmap, so this footprint
  // is transformed into the costmap global frame before Nav2 clips the update.
  *min_x = std::min(*min_x, map_min_x);
  *min_y = std::min(*min_y, map_min_y);
  *max_x = std::max(*max_x, map_max_x);
  *max_y = std::max(*max_y, map_max_y);

  current_ = true;
}

void ElevationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  grid_map_msgs::msg::GridMap::SharedPtr map;
  size_t received_map_count = 0;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = latest_map_;
    received_map_count = received_map_count_;
  }

  if (!map) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *clock_,
      5000,
      "ElevationLayer '%s' updateCosts skipped: no GridMap received yet on '%s'.",
      name_.c_str(),
      elevation_topic_.c_str());
    return;
  }

  size_t elevation_layer_index = 0;
  size_t traversability_layer_index = 0;
  if (needsElevationLayer() && !getLayerIndex(*map, elevation_layer_name_, elevation_layer_index)) {
    return;
  }
  if (needsTraversabilityLayer() &&
    !getLayerIndex(*map, traversability_layer_name_, traversability_layer_index))
  {
    return;
  }

  const std::string costmap_frame = normalizeFrameId(layered_costmap_->getGlobalFrameID());
  const std::string gridmap_frame = getGridMapFrame(*map, costmap_frame);
  geometry_msgs::msg::TransformStamped costmap_to_gridmap;
  if (!lookupTransformToFrame(gridmap_frame, costmap_frame, costmap_to_gridmap)) {
    current_ = false;
    return;
  }

  const int start_i = std::max(0, min_i);
  const int start_j = std::max(0, min_j);
  const int end_i = std::min(max_i, static_cast<int>(master_grid.getSizeInCellsX()));
  const int end_j = std::min(max_j, static_cast<int>(master_grid.getSizeInCellsY()));
  if (start_i >= end_i || start_j >= end_j) {
    return;
  }

  size_t traversed_cells = 0;
  size_t ordinary_writes = 0;
  size_t lethal_writes = 0;
  size_t skipped_unknown = 0;
  size_t out_of_bounds = 0;
  size_t transform_failures = 0;
  size_t step_limited_writes = 0;

  for (int mx = start_i; mx < end_i; ++mx) {
    for (int my = start_j; my < end_j; ++my) {
      ++traversed_cells;

      const auto cell_x = static_cast<unsigned int>(mx);
      const auto cell_y = static_cast<unsigned int>(my);
      const auto old_cost = master_grid.getCost(cell_x, cell_y);
      if (old_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }

      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(cell_x, cell_y, wx, wy);

      double gridmap_wx = 0.0;
      double gridmap_wy = 0.0;
      if (!transformPoint2D(costmap_to_gridmap, wx, wy, gridmap_wx, gridmap_wy)) {
        ++transform_failures;
        continue;
      }

      unsigned int gx = 0;
      unsigned int gy = 0;
      if (!worldToGridMapIndex(*map, gridmap_wx, gridmap_wy, gx, gy)) {
        ++out_of_bounds;
        if (unknown_as_obstacle_) {
          master_grid.setCost(cell_x, cell_y, nav2_costmap_2d::NO_INFORMATION);
          ++skipped_unknown;
        }
        continue;
      }

      unsigned char new_cost = nav2_costmap_2d::FREE_SPACE;
      bool step_limited = false;
      if (!computeCostAtGridMapIndex(
          *map,
          elevation_layer_index,
          traversability_layer_index,
          gx,
          gy,
          new_cost,
          step_limited))
      {
        ++skipped_unknown;
        if (unknown_as_obstacle_) {
          master_grid.setCost(cell_x, cell_y, nav2_costmap_2d::NO_INFORMATION);
        }
        continue;
      }

      if (new_cost == nav2_costmap_2d::NO_INFORMATION) {
        if (unknown_as_obstacle_) {
          master_grid.setCost(cell_x, cell_y, nav2_costmap_2d::NO_INFORMATION);
          ++skipped_unknown;
        }
        continue;
      }

      if (new_cost == nav2_costmap_2d::FREE_SPACE) {
        continue;
      }

      if (new_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
        master_grid.setCost(cell_x, cell_y, nav2_costmap_2d::LETHAL_OBSTACLE);
        ++lethal_writes;
        if (step_limited) {
          ++step_limited_writes;
        }
        continue;
      }

      if (old_cost == nav2_costmap_2d::NO_INFORMATION || new_cost > old_cost) {
        master_grid.setCost(cell_x, cell_y, new_cost);
        ++ordinary_writes;
      }
    }
  }

  RCLCPP_INFO_THROTTLE(
    logger_,
    *clock_,
    5000,
    "ElevationLayer '%s' updateCosts: received_maps=%zu cost_source='%s' "
    "elevation_layer_index=%zu traversability_layer_index=%zu cells=%zu "
    "ordinary=%zu lethal=%zu skipped_unknown=%zu out_of_bounds=%zu "
    "transform_failures=%zu step_limited=%zu costmap_frame='%s' gridmap_frame='%s'.",
    name_.c_str(),
    received_map_count,
    cost_source_name_.c_str(),
    elevation_layer_index,
    traversability_layer_index,
    traversed_cells,
    ordinary_writes,
    lethal_writes,
    skipped_unknown,
    out_of_bounds,
    transform_failures,
    step_limited_writes,
    costmap_frame.c_str(),
    gridmap_frame.c_str());

}

void ElevationLayer::reset()
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  latest_map_.reset();
  received_map_count_ = 0;
  current_ = true;
}

bool ElevationLayer::isClearable()
{
  return false;
}

}  // namespace elevation_nav2_bridge

PLUGINLIB_EXPORT_CLASS(elevation_nav2_bridge::ElevationLayer, nav2_costmap_2d::Layer)
