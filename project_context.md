# 项目背景

## 机器人类型

当前项目面向 ROS2 Humble + Nav2 移动机器人导航系统，运行平台是 NVIDIA Jetson。机器人使用 Nav2 做导航，并希望把 2.5D 高程信息接入 costmap，让路径规划避开高度突变、坡、凸起等地形风险。

## 使用场景

当前测试阶段没有 Gazebo，主要使用：

- `elevation_mapping_cupy` 的 synthetic demo 发布假点云和高程图。
- RViz 查看 `/elevation_traversability_debug` 和 `/global_costmap/costmap`。
- Nav2 在宿主机运行。
- `elevation_mapping_cupy` 在 Docker 中运行。

## 当前目标

当前目标不是完整地形导航算法，而是先完成最小闭环：

```text
Docker elevation_mapping_cupy
  -> /elevation_mapping_node/elevation_map
  -> elevation_nav2_bridge::ElevationLayer
  -> Nav2 global_costmap master_grid
  -> /global_costmap/costmap
  -> RViz / Nav2 global planner
```

## 为什么要做 elevation map 接 Nav2

Nav2 默认 costmap 主要处理 2D 障碍物。高程图是 2.5D 数据，不能直接给 Nav2 用。需要一个 costmap layer plugin 把 GridMap 里的高程信息转换成 Nav2 可理解的 cost：

- 高度低：free 或低 cost
- 高度高：lethal obstacle
- 中间高度：按比例转换成普通 cost
- 后续再扩展 slope / roughness / traversability

# 当前系统环境

## Jetson / Ubuntu / ROS2 / Docker / Nav2

已知环境：

- Jetson：NVIDIA Jetson，用户终端显示 `nvidia@nvidia-desktop`
- ROS2：Humble
- Nav2：宿主机运行
- elevation_mapping_cupy：Docker 中运行
- `elevation_nav2_bridge`：必须放在宿主机 Nav2 工作空间编译，不能放进 Docker

重要原则：

- Docker 只跑 `elevation_mapping_cupy`
- 宿主机跑 Nav2 和 `elevation_nav2_bridge`
- bridge 通过 ROS2 topic 订阅 Docker 发布的高程图

## 宿主机和 Docker 路径

宿主机 Nav2 工作空间：

```bash
~/navigation_test/ros2bookcode/chapt7/chapt7_ws
```

宿主机 bridge 包路径：

```bash
~/navigation_test/ros2bookcode/chapt7/chapt7_ws/src/elevation_nav2_bridge
```

Docker 中 elevation map 运行目录，用户测试时出现过：

```bash
~/elevation_map
```

当前 Codex 工作区镜像路径：

```text
e:\ElevationMap
```

## Docker 和宿主机分工

Docker 内：

- 运行 `elevation_mapping_cupy`
- 发布点云输入和高程图输出
- synthetic demo 输出 topic：

```bash
/camera/depth/points
/elevation_mapping_node/elevation_map
/tf
/tf_static
```

宿主机：

- 编译 `elevation_nav2_bridge`
- 启动 Nav2
- global_costmap 加载 `elevation_nav2_bridge::ElevationLayer`
- RViz 查看 costmap 和 debug grid

# 当前代码结构

## 工作空间路径

主要工作区：

```text
src/elevation_mapping_cupy
src/elevation_nav2_bridge
```

## 关键包名

已有包：

- `elevation_mapping_cupy`

新增包：

- `elevation_nav2_bridge`

## 关键文件

bridge 插件：

```text
src/elevation_nav2_bridge/package.xml
src/elevation_nav2_bridge/CMakeLists.txt
src/elevation_nav2_bridge/plugins/elevation_layer.xml
src/elevation_nav2_bridge/include/elevation_nav2_bridge/elevation_layer.hpp
src/elevation_nav2_bridge/src/elevation_layer.cpp
```

synthetic demo：

```text
src/elevation_mapping_cupy/elevation_mapping_cupy/launch/synthetic_depth_demo.launch.py
src/elevation_mapping_cupy/elevation_mapping_cupy/scripts/synthetic_pointcloud_tf_publisher.py
src/elevation_mapping_cupy/elevation_mapping_cupy/config/setups/synthetic/synthetic_depth.yaml
src/elevation_mapping_cupy/elevation_mapping_cupy/config/core/core_param.yaml
```

## launch 文件

当前主要使用：

```bash
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py
```

该 launch 当前做了这些事：

- 启动 `synthetic_pointcloud_tf_publisher.py`
- 启动 `elevation_mapping_node.py`
- 可选启动 RViz
- 增加了 fake `map -> odom` 静态 TF
- synthetic publisher 改成发布 `odom -> base_link`

关键参数：

```bash
use_sim_time:=false
launch_rviz:=false
publish_map_to_odom:=true
```

如果 AMCL 或其他定位已经发布 `map -> odom`，要用：

```bash
publish_map_to_odom:=false
```

## config 文件

synthetic 高程配置：

```text
src/elevation_mapping_cupy/elevation_mapping_cupy/config/setups/synthetic/synthetic_depth.yaml
```

其中关键输出：

```yaml
publishers:
  elevation_map:
    layers: ["elevation", "variance", "traversability"]
    basic_layers: ["elevation"]
    fps: 10.0
```

核心参数：

```text
src/elevation_mapping_cupy/elevation_mapping_cupy/config/core/core_param.yaml
```

关键值：

```yaml
resolution: 0.1
map_length: 20.0
map_frame: 'odom'
base_frame: 'base_link'
corrected_map_frame: 'odom'
```

synthetic setup 会覆盖为：

```yaml
map_frame: "map"
base_frame: "base_link"
corrected_map_frame: "map"
```

## plugin.xml / CMakeLists / package.xml 注意点

插件类名：

```cpp
elevation_nav2_bridge::ElevationLayer
```

插件导出：

```cpp
PLUGINLIB_EXPORT_CLASS(elevation_nav2_bridge::ElevationLayer, nav2_costmap_2d::Layer)
```

插件 XML：

```text
src/elevation_nav2_bridge/plugins/elevation_layer.xml
```

CMake 里需要：

```cmake
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins/elevation_layer.xml)
```

依赖：

```text
rclcpp
nav2_costmap_2d
pluginlib
grid_map_msgs
nav_msgs
geometry_msgs
tf2_ros
```

# 已经实现的功能

## elevation_nav2_bridge 插件

已实现：

- ROS2 `ament_cmake` C++ package
- Nav2 costmap layer plugin
- 类名：`elevation_nav2_bridge::ElevationLayer`
- 继承：`nav2_costmap_2d::Layer`
- 已实现：
  - `onInitialize()`
  - `updateBounds()`
  - `updateCosts()`
  - `reset()`
  - `isClearable()`
- pluginlib 导出
- 已能在 Jetson 上 `colcon build` 通过早期版本

当前最新代码又修改了 GridMap 坐标映射，需要在 Jetson 上重新编译确认。

## topic 订阅

bridge 默认订阅：

```bash
/elevation_mapping_node/elevation_map
```

消息类型：

```bash
grid_map_msgs/msg/GridMap
```

用户已经验证过：

```bash
ros2 topic info /elevation_mapping_node/elevation_map -v
```

显示：

```text
Type: grid_map_msgs/msg/GridMap
Publisher count: 1
Subscription count: 1
Node name: global_costmap
Node namespace: /global_costmap
```

说明 global_costmap 内的插件已经订阅到了高程图。

## updateBounds / updateCosts

已实现并被 Nav2 调用。

`updateBounds()` 当前逻辑：

- 如果 `enabled=false`，直接 return
- 如果没收到 map，直接 return
- 根据最新 GridMap 覆盖范围扩大 update bounds
- 最新代码使用 GridMap pose + yaw 计算 bounds

`updateCosts()` 当前逻辑：

- 如果 `enabled=false`，直接 return
- 如果还没收到 GridMap，throttled warn 后 return
- 查找 `elevation_layer_name`
- 遍历 Nav2 master_grid 更新窗口
- `master_grid.mapToWorld()` 得到世界坐标
- 映射到 GridMap index
- 读取 elevation
- 转换成 cost
- 写入 `master_grid`

## cost 转换完成度

当前只实现 height cost：

- 不做 slope
- 不做 roughness
- 不做完整 traversability
- 只读取 GridMap 的 `elevation` layer

规则：

```text
elevation >= lethal_height_threshold -> LETHAL_OBSTACLE 254
elevation <= min_height              -> FREE_SPACE 0
中间高度                              -> 1~252
NaN/inf                              -> 跳过，或 unknown_as_obstacle=true 时写 NO_INFORMATION 255
```

## debug grid

bridge 会发布：

```bash
/elevation_traversability_debug
```

类型：

```bash
nav_msgs/msg/OccupancyGrid
```

用途：

- 只用于调试 height -> cost 是否正常
- 不是 Nav2 真正用于规划的 costmap
- 真正用于 Nav2 的是 `/global_costmap/costmap`

当前最新代码中，debug grid 已改为按世界坐标重采样 GridMap，而不是直接把 GridMap 数组当普通 OccupancyGrid 贴出来。

## global_costmap 状态

已经做到：

- `global_costmap` 能加载 elevation layer
- bridge 能订阅 elevation map
- `/global_costmap/costmap` 能看到 elevation 生成的 cost
- RViz 能看到高程 cost

注意：

- 如果 `/global_costmap/costmap` 没发布，先查 Nav2 lifecycle、TF、`use_sim_time`、参数是否生效。
- 如果只看 `/elevation_traversability_debug`，只能证明 bridge 收到了 GridMap 并能转换 debug，不等于 Nav2 master_grid 一定写成功。

# 当前关键参数

## 推荐 global_costmap 测试配置

当前调试阶段建议先只开 elevation layer，不开 static/obstacle/inflation：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      always_send_full_costmap: true

      rolling_window: false
      width: 40
      height: 40
      origin_x: -20.0
      origin_y: -20.0

      plugins: ["elevation_layer"]

      elevation_layer:
        plugin: "elevation_nav2_bridge::ElevationLayer"
        enabled: true
        elevation_topic: "/elevation_mapping_node/elevation_map"
        elevation_layer_name: "elevation"
        min_height: 0.0
        lethal_height_threshold: 0.25
        cost_scale: 252.0
        unknown_as_obstacle: false
        transform_tolerance: 0.2
        publish_debug_grid: true
        debug_grid_topic: "/elevation_traversability_debug"
```

## 参数解释

`global_frame`

- global costmap 固定坐标系
- 当前用 `map`

`robot_base_frame`

- 机器人底盘坐标系
- 当前用 `base_link`

`use_sim_time`

- 当前没有 Gazebo 和 `/clock`
- 必须用 `false`
- 如果 Nav2 是 `true` 但没有 `/clock`，costmap timer 可能不发布

`rolling_window`

- `false`：global_costmap 固定在 map 坐标
- `true`：costmap 窗口跟着机器人走
- 当前要调全局地图，不建议开 rolling window

`width` / `height`

- 当前用户 Jetson 上 `ros2 param get` 显示是 integer
- 在 YAML 中建议写整数，例如 `40`，不要写 `40.0`

`origin_x` / `origin_y`

- fixed global costmap 的左下角世界坐标
- 如果 `width: 40`、`height: 40`、`origin_x: -20.0`、`origin_y: -20.0`，范围是：

```text
x: [-20, 20]
y: [-20, 20]
```

`resolution`

- Nav2 costmap 分辨率
- 当前常用 `0.05`
- elevation map 分辨率是 `0.1`

`elevation_topic`

- Docker 里的 `elevation_mapping_cupy` 当前发布：

```bash
/elevation_mapping_node/elevation_map
```

不是 `/elevation_map`

`elevation_layer_name`

- 当前读取 GridMap 中的：

```text
elevation
```

`min_height`

- 低于该高度视为 free

`lethal_height_threshold`

- 高于该高度视为 lethal obstacle
- 当前测试值 `0.25`

`cost_scale`

- 中间高度映射到普通 cost 的最大比例
- 当前 `252.0`
- 不输出 253，253 是 Nav2 inscribed inflated obstacle

`unknown_as_obstacle`

- `false`：无数据跳过，不改 master_grid
- `true`：无数据写 NO_INFORMATION

`transform_tolerance`

- TF 查询超时时间，单位秒
- 当前建议 `0.2`
- 如果 GridMap frame 和 costmap global_frame 相同，则不查 TF，使用 identity transform
- 如果 frame 不同，例如 local_costmap 是 `odom` 而 GridMap 是 `map`，bridge 会用 TF 把 costmap cell 坐标转换到 GridMap frame 后再查 elevation

`publish_debug_grid`

- 是否发布 `/elevation_traversability_debug`
- 当前建议 `true`

# 已经踩过的坑

## Docker 停了但 topic 还在

ROS2 topic 看起来“残留”通常是：

- Docker 容器其实还在跑
- 宿主机 ROS2 节点没死
- `ros2 daemon` 缓存没刷新
- RViz 显示了 transient local 的最后一帧

不要粗暴使用：

```bash
docker stop $(docker ps -q)
```

这会停掉所有容器。

正确做法：

```bash
docker ps --format '{{.ID}} {{.Names}} {{.Image}}'
docker ps --format '{{.ID}} {{.Names}} {{.Image}}' | grep -i elevation
docker stop <elevation相关容器ID或名字>
ros2 daemon stop
ros2 daemon start
```

## 容器里 code 命令不存在

Docker 容器中通常没有 VS Code 的 `code` 命令。要么在宿主机打开挂载目录，要么用终端工具查看文件。不要把 bridge 放进 Docker。

## global_costmap 参数没生效

出现过参数没生效或 `/global_costmap/costmap` 不发布的情况。

检查参数：

```bash
ros2 param get /global_costmap/global_costmap width
ros2 param get /global_costmap/global_costmap height
ros2 param get /global_costmap/global_costmap origin_x
ros2 param get /global_costmap/global_costmap origin_y
ros2 param get /global_costmap/global_costmap resolution
ros2 param get /global_costmap/global_costmap rolling_window
```

用户发现当前环境中 `width` / `height` 是 integer。YAML 里写 `40` 比 `40.0` 更稳。

## costmap origin 实际是哪里

固定窗口 costmap：

```yaml
rolling_window: false
width: 40
height: 40
origin_x: -20.0
origin_y: -20.0
```

则 costmap 左下角是 `(-20, -20)`，不是机器人位置。RViz 里看到的灰色大区域应固定在 map。

## RViz 看到的 costmap 为什么不是 -10 到 10

原因可能是：

- 参数没有真正加载
- `width/height` 类型不匹配
- 使用了默认 `width: 5` / `height: 5`
- 打开的是 local costmap 或 downsampled costmap，不是 global costmap
- RViz 显示的是旧 topic 或旧配置

## global_costmap / local_costmap 坐标系问题

global costmap：

- 通常 `global_frame: map`
- 应该固定在 map
- 不应该跟机器人走，除非 `rolling_window: true`

local costmap：

- 通常 `global_frame: odom`
- 通常 `rolling_window: true`
- 跟机器人走是正常的

当前高程接入先验证 global_costmap，后续再迁移到 local_costmap 做局部地形判断。

## elevation map 只是 2.5D 可视化，不等于 Nav2 costmap

`/elevation_mapping_node/elevation_map`：

- GridMap
- 2.5D 高程图
- Nav2 不直接使用

`/elevation_traversability_debug`：

- bridge 发布的 debug OccupancyGrid
- 用来检查 height -> cost 转换
- 不用于 Nav2 规划

`/global_costmap/costmap`：

- Nav2 真正发布的 global costmap
- global planner 用它规划

## synthetic demo 的 TF 问题

曾经重复启动导致多个同名节点：

```text
/synthetic_pointcloud_tf_publisher
/synthetic_pointcloud_tf_publisher
...
```

会导致 TF 跳变。清理：

```bash
pkill -f synthetic_pointcloud_tf_publisher
pkill -f elevation_mapping_node
pkill -f rviz2
pkill -f static_transform_publisher
ros2 daemon stop
ros2 daemon start
```

## use_sim_time 问题

当前没有 Gazebo，没有 `/clock`。因此：

```yaml
use_sim_time: false
```

如果 elevation 用 false，而 Nav2 用 true，Nav2 的 costmap timer 可能不动，`/global_costmap/costmap` 可能不发布。

## GridMap 坐标映射问题

之前 bridge 把 GridMap 当普通 OccupancyGrid 处理，使用：

```text
origin_x = center_x - length_x / 2
origin_y = center_y - length_y / 2
index = floor((world - origin) / resolution)
```

这对 `grid_map_msgs/GridMap` 不严格正确，因为 GridMap 的 row/column 方向和普通 OccupancyGrid 不一样。

当前最新代码已改：

- 使用 GridMap center
- 使用 GridMap yaw
- 按 grid_map row/column 约定计算 index
- debug grid 也按世界坐标重采样

该修改需要在 Jetson 上重新 build 和验证。

## ElevationLayer 的 TF frame transform 方案

本轮已按该方案在 `elevation_nav2_bridge::ElevationLayer` 中实现 TF frame transform。

当前 elevation layer 已经能在 synthetic 环境中写入 `global_costmap` 和 `local_costmap`。但早期实现默认：

```text
Nav2 costmap global_frame 坐标
  == GridMap header.frame_id / GridMap info.pose 所在坐标
```

这在 synthetic demo 中通常成立，因为 `map -> odom` 是静态 identity。真实导航时，如果 AMCL 或其他定位源发布非 identity 的 `map -> odom`，则可能出现：

```text
local_costmap global_frame: odom
GridMap frame: map
map -> odom 非 identity
```

这种情况下，不能直接把 `master_grid.mapToWorld()` 得到的 `odom` 坐标拿去查 `map` frame 下的 GridMap。需要在 bridge 内显式做 TF。

目标数据流：

```text
local/global costmap cell
  -> master_grid.mapToWorld()
  -> 得到 costmap_frame 下的坐标
  -> TF 转到 GridMap frame
  -> worldToGridMapIndex()
  -> 读取 elevation
  -> height -> cost
  -> master_grid.setCost()
```

`updateCosts()` 的 TF 逻辑：

```text
costmap_frame = layered_costmap_->getGlobalFrameID()
gridmap_frame = latest_map_->header.frame_id

lookupTransform(
  target_frame = gridmap_frame,
  source_frame = costmap_frame
)

对每个 master_grid cell：
  cell index -> costmap_frame 下的 wx/wy
  wx/wy -> gridmap_frame 下的 gx_world/gy_world
  gx_world/gy_world -> GridMap index
  elevation -> Nav2 cost
```

`updateBounds()` 的 TF 逻辑：

```text
GridMap 四个角点先在 gridmap_frame 中计算
  -> TF 转到 costmap_frame
  -> 用转换后的四个角扩展 min_x/min_y/max_x/max_y
```

这样 Nav2 才会在正确的 local/global costmap 区域调用 `updateCosts()`。

实现原则：

- 仍然保持 layer 是实时局部 GridMap 查询型 costmap layer，不做全局持久化。
- 不把 GridMap 整体转换成新地图，避免额外拷贝和重采样。
- 每次 `updateCosts()` 只 lookup 一次 TF，循环内只做轻量点坐标变换。
- 如果 GridMap frame 和 costmap frame 相同，则使用 identity transform，不查 TF。
- 如果 GridMap header frame 为空，则退回 costmap frame，并打印 throttled warning。
- 如果查不到 TF，则本轮 `updateBounds()` / `updateCosts()` 跳过，并将 layer 标记为非 current。
- 新增参数 `transform_tolerance`，默认 `0.2` 秒。
- 当前实现使用 `tf2::TimePointZero` 查询最新 TF，优先保证 local_costmap 实时运行稳定；如果后续需要严格时间同步，可以改为按 `GridMap.header.stamp` 查询。

和 Nav2 自带 layer 的关系：

- `ObstacleLayer` / `VoxelLayer` 是“传感器点 -> TF -> mark/clear costmap”。
- `StaticLayer` 在 rolling local costmap 场景下会把 costmap cell 转到 map frame 查询静态地图。
- 当前 `ElevationLayer` 语义上是实时局部地形层，但数据结构是 GridMap patch，因此更适合采用“costmap cell -> GridMap frame 查询”的模式。

验证方法：

1. 先在 synthetic identity TF 环境下验证，结果应与未加 TF 时一致。
2. 再故意发布非 identity 的 `map -> odom`，例如平移 1m，观察 cost 是否仍能对齐。
3. 检查日志中的 `costmap_frame`、`gridmap_frame`、`transform_failures`、`ordinary`、`lethal`、`out_of_bounds`。

# 常用验证命令

## 编译 bridge

```bash
cd ~/navigation_test/ros2bookcode/chapt7/chapt7_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select elevation_nav2_bridge --symlink-install
source install/setup.bash
```

## 启动 synthetic elevation demo

无 AMCL / 无定位时：

```bash
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py \
  launch_rviz:=false \
  use_sim_time:=false \
  publish_map_to_odom:=true
```

已有 AMCL 或其他 map->odom 时：

```bash
ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py \
  launch_rviz:=false \
  use_sim_time:=false \
  publish_map_to_odom:=false
```

## topic 检查

```bash
ros2 topic list
ros2 topic info /elevation_mapping_node/elevation_map -v
ros2 topic echo --once /elevation_mapping_node/elevation_map --field layers
ros2 topic info /elevation_traversability_debug
ros2 topic echo --once /elevation_traversability_debug --field info
ros2 topic info /global_costmap/costmap -v
ros2 topic echo --once /global_costmap/costmap --field info
ros2 topic hz /global_costmap/costmap
```

注意不要拼错 topic，例如曾经输过：

```bash
/global_costmap/costmapap
```

这是错的。

## 参数检查

```bash
ros2 param get /global_costmap/global_costmap width
ros2 param get /global_costmap/global_costmap height
ros2 param get /global_costmap/global_costmap origin_x
ros2 param get /global_costmap/global_costmap origin_y
ros2 param get /global_costmap/global_costmap resolution
ros2 param get /global_costmap/global_costmap rolling_window
ros2 param get /global_costmap/global_costmap use_sim_time
```

## TF 检查

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
```

如果提示多个同名节点，说明重复 launch 没清干净。

## Docker 检查

```bash
docker ps --format '{{.ID}} {{.Names}} {{.Image}}'
docker ps --format '{{.ID}} {{.Names}} {{.Image}}' | grep -i elevation
docker inspect <container_name_or_id>
```

只停 elevation 相关容器：

```bash
docker stop <elevation相关容器ID或名字>
```

## 清理 ROS2 残留

谨慎清理：

```bash
pkill -f synthetic_pointcloud_tf_publisher
pkill -f elevation_mapping_node
pkill -f rviz2
pkill -f static_transform_publisher
ros2 daemon stop
ros2 daemon start
```

# 当前判断标准

## 插件是否加载成功

看 Nav2 启动日志，应该看到 `ElevationLayer` 初始化日志，例如：

```text
Initialized ElevationLayer 'elevation_layer'
```

也可以看 topic subscription：

```bash
ros2 topic info /elevation_mapping_node/elevation_map -v
```

如果里面出现：

```text
Node name: global_costmap
Node namespace: /global_costmap
Endpoint type: SUBSCRIPTION
```

说明插件在 global_costmap 进程里订阅成功。

## 是否收到 elevation map

```bash
ros2 topic info /elevation_mapping_node/elevation_map -v
```

应该：

```text
Publisher count: 1
Subscription count: 1
```

并且：

```bash
ros2 topic echo --once /elevation_mapping_node/elevation_map --field layers
```

应包含：

```text
elevation
variance
traversability
```

## updateCosts 是否写入 master_grid

bridge 有 throttled 日志：

```text
ElevationLayer 'elevation_layer' updateCosts:
received_maps=...
layer_index=...
cells=...
ordinary=...
lethal=...
skipped_unknown=...
out_of_bounds=...
```

如果 `ordinary` 或 `lethal` 大于 0，说明有写入。

如果 `out_of_bounds` 很大，说明 global_costmap 窗口和 elevation map 覆盖区域没对上，或者坐标映射有问题。

## /global_costmap/costmap 的 info 是否符合预期

检查：

```bash
ros2 topic echo --once /global_costmap/costmap --field info
```

期望：

- `resolution` 与 YAML 一致
- `width` / `height` 与 YAML 对应
- `origin.position.x/y` 与 YAML 一致
- 如果 `rolling_window: false`，origin 不应该跟机器人移动

## RViz 应该看什么

RViz 中：

- Fixed Frame：`map`
- 看 `/global_costmap/costmap`
- 这是 Nav2 真正用于规划的 costmap
- 高程 cost 应该出现在 map 中正确位置

如果开启 `/elevation_traversability_debug`：

- 它是调试图
- 只代表当前最新 elevation map 转换结果
- 它可能表现为局部 patch，不能等同于 global_costmap

# 下一步开发计划

1. 先确认 YAML 参数真正生效

   - 用 `ros2 param get` 检查 `width`、`height`、`origin_x`、`origin_y`、`rolling_window`、`use_sim_time`
   - 确认 `width/height` 用整数

2. 确认 global_costmap 范围

   - 用 `/global_costmap/costmap --field info` 看 origin 和尺寸
   - 确认机器人和 elevation map patch 在 global_costmap 范围内

3. 确认 elevation_layer 写入 cost

   - 看 bridge 的 `updateCosts` 日志
   - 看 `ordinary/lethal/out_of_bounds`
   - RViz 看 `/global_costmap/costmap`

4. 增强 debug grid

   - 当前已有 `/elevation_traversability_debug`
   - 后续可增加额外 debug topic，例如 raw height debug、index mapping debug、bounds marker

5. 从 height cost 改成 slope / traversability cost

   - 当前只用 `elevation`
   - 下一步可读取 `traversability`
   - 或在 bridge 内计算 slope

6. 后续考虑 layer 组合

   推荐顺序：

   ```yaml
   plugins: ["static_layer", "obstacle_layer", "elevation_layer", "inflation_layer"]
   ```

   当前调试 raw elevation cost 时先不要开 inflation，否则看不清原始高程 cost。

7. 最终迁移到 local_costmap 做局部 2.5D 可通行性判断

   - global_costmap 用于全局规划
   - local_costmap 用于近场避障和局部控制
   - 真实地形风险更适合 local_costmap 做实时判断

8. 后续如果需要全局持久高程代价

   当前 bridge 只是把最新局部 elevation map overlay 到 Nav2 master_grid，不是长期全局融合地图。

   如果要“走过的地方累计成全局高程 costmap”，需要新增持久化 buffer：

   - 固定 global grid
   - 每帧 elevation 写入全局 buffer
   - 数据过期/清除策略
   - 再把 buffer 写入 master_grid

# 给新对话的接续提示词

请复制下面这段到新 ChatGPT 对话：

```text
你现在继续作为 ROS2 Humble + Nav2 + elevation_mapping_cupy 项目助手。

请先阅读我提供的 project_context.md，不要从零解释 ROS2/Nav2。当前项目已经有：

1. Docker 中运行 elevation_mapping_cupy，发布 /elevation_mapping_node/elevation_map，类型 grid_map_msgs/msg/GridMap。
2. Jetson 宿主机 Nav2 工作空间中新增 package：src/elevation_nav2_bridge。
3. 插件 elevation_nav2_bridge::ElevationLayer 已实现为 Nav2 costmap layer plugin。
4. 插件能订阅 /elevation_mapping_node/elevation_map，并把 GridMap 的 elevation layer 转成 Nav2 cost 写入 global_costmap master_grid。
5. 已有 debug topic：/elevation_traversability_debug，类型 nav_msgs/msg/OccupancyGrid。
6. 当前正在调试 global_costmap 固定窗口、GridMap 坐标映射、RViz 中 costmap 是否固定在 map 坐标。

重要限制：
- 不要把 bridge 放进 Docker。
- 不要修改 navigation2 源码。
- 不要重构整个项目。
- 当前阶段只先调通 height -> cost，不要一次性实现 slope/roughness/traversability。
- 当前没有 Gazebo，没有 /clock，因此 use_sim_time 应为 false。

请基于 project_context.md 继续帮我调试/写代码/分析问题，优先检查：
1. YAML 参数是否真正生效。
2. /global_costmap/costmap 的 info 是否符合 origin/width/height。
3. elevation_layer 的 updateCosts 是否写入 ordinary/lethal cost。
4. GridMap 到 Nav2 costmap 的坐标映射是否正确。
```
