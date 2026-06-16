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

5. 从 height cost 扩展到 traversability / fused cost

   - 当前 bridge 已支持 `cost_source: "elevation" | "traversability" | "fused"`
   - `elevation`：只按 GridMap 的 `elevation` 高度转 cost
   - `traversability`：只按 cupy 发布的 `traversability` 转 cost
   - `fused`：同时读取 `traversability` 和 `elevation`，把 cupy 可通行性代价与局部台阶高度代价取更危险的那个

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

# 当前四轮足 traversability/fused cost 方案

当前实现的第一版可通行性转换在 `elevation_nav2_bridge::ElevationLayer` 内完成，不修改 Nav2 源码，也不修改 `elevation_mapping_cupy`。

## 数据来源

输入仍然是：

```text
/elevation_mapping_node/elevation_map
grid_map_msgs/msg/GridMap
```

要求 GridMap 至少包含：

```text
elevation
traversability
```

其中：

- `elevation` 是每个 GridMap cell 的高度，单位 m。
- `traversability` 是 `elevation_mapping_cupy` 内部 traversability filter 输出的可通行分数，通常按 0 到 1 理解，越大越可通行。

## cost_source 三种模式

`ElevationLayer` 新增参数：

```yaml
cost_source: "elevation"       # 只用高度
cost_source: "traversability"  # 只用 cupy traversability
cost_source: "fused"           # traversability + 台阶高度检查
```

默认仍是 `"elevation"`，这样旧 YAML 不会被破坏。

当前局部测试配置 `local_costmap_elevation_only.yaml` 已切到：

```yaml
cost_source: "fused"
```

## traversability 到 Nav2 cost 的规则

参数：

```yaml
free_traversability_threshold: 0.8
lethal_traversability_threshold: 0.25
traversability_cost_scale: 120.0
```

转换逻辑：

- `traversability >= 0.8`：认为可通行，写 `FREE_SPACE`
- `traversability <= 0.25`：认为风险已满，但只写 ordinary cost，不直接 lethal
- 中间区间：线性映射到 Nav2 ordinary cost，越接近 0.25 cost 越高
- `traversability_cost_scale` 控制这个辅助风险最高能加多少 cost；当前 local 测试配置为 120
- `lethal_traversability_threshold` 是历史参数名；当前语义更接近 `full_risk_traversability_threshold`，不会单独触发 lethal

也就是说，cupy 的 traversability 不是直接当 Nav2 cost 用，也不再作为硬物理限制。它现在只作为辅助风险评分；真正的 lethal 由明确物理量触发，例如台阶高度、下落高度、后续的坡度上限等。

## 15cm 台阶限制

四轮足当前先按“最高可跨越 15cm”做硬限制。

参数：

```yaml
enable_step_height_check: true
max_step_height: 0.15
comfortable_step_height: 0.06
max_drop_height: 0.12
step_neighbor_radius: 1
```

每个 GridMap cell 会读取它周围 `step_neighbor_radius` 范围内的邻居高度。默认 `1` 表示 3x3 邻域。

对当前 cell：

- 邻居比当前 cell 高，记为 `step_up`
- 邻居比当前 cell 低，记为 `step_down`

规则：

- `step_up > 0.15m`：直接 lethal
- `step_down > 0.12m`：直接 lethal
- `step_up/step_down <= 0.06m`：不额外加台阶 cost
- `0.06m ~ 0.15m`：线性增加 ordinary cost

这不是完整的足端落脚点评估，只是第一版局部几何限制：先把明显超过机器人能力的高度突变从局部代价地图里打成不可通行。

## fused 模式最终写入 master_grid 的规则

`cost_source: "fused"` 时：

1. costmap cell 坐标先通过 TF 转到 GridMap frame。
2. `worldToGridMapIndex()` 找到对应 GridMap cell。
3. 读取 `traversability`，计算 traversability cost。
4. 读取 `elevation` 和邻居 elevation，计算局部台阶 cost。
5. 如果台阶超过硬限制，直接写 `LETHAL_OBSTACLE`。
6. 否则取：

```text
final_cost = max(traversability_cost, step_cost)
```

再写入 Nav2 `master_grid`。

注意：当前 traversability cost 是 soft risk，最多影响路径偏好，不会单独写 `LETHAL_OBSTACLE`。如果某个 cell lethal，应该来自 step/drop 等物理硬限制，或者后续新增的 slope/stair hard limit。

## 运行中动态调参

`ElevationLayer` 已支持运行中修改关键限制参数，不需要重启 Nav2。

常用命令：

```bash
ros2 param set /local_costmap/local_costmap elevation_layer.max_step_height 0.18
ros2 param set /local_costmap/local_costmap elevation_layer.max_drop_height 0.12
ros2 param set /local_costmap/local_costmap elevation_layer.comfortable_step_height 0.06
ros2 param set /local_costmap/local_costmap elevation_layer.traversability_cost_scale 80.0
ros2 param set /local_costmap/local_costmap elevation_layer.enable_step_height_check true
```

含义：

- `max_step_height`：向上台阶硬限制，超过后 lethal
- `max_drop_height`：向下落差硬限制，超过后 lethal
- `comfortable_step_height`：舒适高度，低于它不额外加 step cost
- `traversability_cost_scale`：cupy traversability 辅助风险强度，不产生 lethal
- `enable_step_height_check`：是否启用 step/drop 物理硬限制

## 日志判断

运行后应该看到类似：

```text
ElevationLayer 'elevation_layer' updateCosts:
received_maps=...
cost_source='fused'
elevation_layer_index=...
traversability_layer_index=...
ordinary=...
lethal=...
step_limited=...
costmap_frame='odom'
gridmap_frame='map'
```

其中：

- `cost_source='fused'` 表示 YAML 生效。
- `traversability_layer_index` 有值，表示找到了 GridMap 的 traversability layer。
- `step_limited > 0` 表示有 cell 是因为超过 `max_step_height` 或 `max_drop_height` 被打成 lethal。
- `transform_failures=0` 表示 TF 转换正常。

## 目前限制

这版还不是完整“爬楼梯策略”，只是 Nav2 costmap 层的可通行性输入。

当前没有做：

- 足端落脚点搜索
- 机身姿态稳定性判断
- 楼梯踏面宽度判断
- 楼梯连续台阶结构识别
- 与局部控制器速度/步态的耦合

但它已经可以让 Nav2 在局部 costmap 上区分：

- 平地：低 cost
- cupy 判断不平/风险高区域：中高 cost
- 超过 15cm 的高度突变：lethal

# synthetic 移动障碍物测试

为了测试“移动障碍物是否能被 elevation_mapping_cupy 识别，并通过 ElevationLayer 写入 local_costmap”，当前在 synthetic 点云源头加入了可开关移动方块。

修改位置：

```text
src/elevation_mapping_cupy/elevation_mapping_cupy/scripts/synthetic_pointcloud_tf_publisher.py
src/elevation_mapping_cupy/elevation_mapping_cupy/launch/synthetic_depth_demo.launch.py
```

核心思路不是直接改 Nav2 costmap，也不是直接发布假 `/elevation_map`，而是从传感器输入侧造一个真实的 `PointCloud2` 障碍物：

```text
moving obstacle points
  -> /camera/depth/points
  -> elevation_mapping_cupy
  -> /elevation_mapping_node/elevation_map
  -> elevation_nav2_bridge::ElevationLayer
  -> /local_costmap/costmap
```

这样测试的是完整链路。

## 当前 moving obstacle 参数

`synthetic_depth_demo.launch.py` 中默认打开：

```yaml
enable_moving_obstacle: true
moving_obstacle_height_m: 0.35
moving_obstacle_size_m: 0.4
moving_obstacle_resolution_m: 0.05
ground_resolution_m: 0.05
moving_obstacle_motion_axis: "lateral"
moving_obstacle_x_m: 1.0
moving_obstacle_y_min_m: -0.8
moving_obstacle_y_max_m: 0.8
moving_obstacle_speed_mps: 0.5
```

含义：

- 方块高度 0.35m，高于当前 `max_step_height: 0.15`，理论上应该触发 lethal 或 `step_limited`
- 方块尺寸 0.4m
- synthetic demo 默认让机器人按方形轨迹运动，并开启 yaw 变化
- 方块固定在机器人前方约 x=1.0m
- 方块沿 y 方向从 -0.8m 到 0.8m 横向往返移动，方便在 3m local costmap 里直接看到运动
- launch 参数里使用 `"lateral"`，不要直接写 `"y"`，否则 ROS2 临时 YAML 可能把 `y` 解析成布尔值 `true`
- `ground_resolution_m` 设为 0.05m。因为 elevation map 分辨率是 0.1m，如果 synthetic 地面点也只按 0.1m 采样，并且机器人静止不动，点云会和 GridMap cell 发生采样相位问题，表现为 `/elevation_mapping_node/elevation_map` 有周期性断裂。地面点云采样加密后，每个 0.1m GridMap cell 更容易收到点，地图会连续很多。

## 观察方法

启动 synthetic demo 和 local costmap 后，在 RViz 看：

```text
/elevation_mapping_node/elevation_map
/local_elevation_traversability_debug
/local_costmap/costmap
```

日志重点看：

```text
cost_source='fused'
ordinary=...
lethal=...
step_limited=...
out_of_bounds=...
transform_failures=0
```

如果移动方块被 fused 规则识别，应该看到：

- RViz 中局部 costmap 的高代价/红色区域随方块移动
- `lethal` 增加
- 如果主要由 15cm 台阶规则触发，`step_limited` 增加

如果方块移走后旧位置仍然长期不消失，问题通常不在 bridge，而在 elevation_mapping_cupy 的地图更新/清除策略：bridge 当前只使用最新一帧 GridMap，不做持久化 buffer。

# 项目总目标和后续研究计划

## 最终目标

本项目的最终目标是：

```text
四轮足机器狗
  -> 输入 MID360 点云
  -> elevation_mapping_cupy 生成 2.5D elevation / traversability GridMap
  -> elevation_nav2_bridge 转成 Nav2 local_costmap 代价
  -> Nav2 结合四轮足运动能力，实现斜坡和楼梯场景下的导航
```

这里的“导航”不是只在 RViz 里看到 costmap，而是至少要达到：

- MID360 实时点云能稳定进入 elevation_mapping_cupy
- GridMap 在 `map/odom/base_link/livox` TF 链下位置正确
- local_costmap 能反映地形可通行性，而不是只做普通 2D 障碍物
- 斜坡不能被误判成墙
- 可跨越台阶不能全部打死
- 超过四轮足能力的台阶、断崖、障碍物要变成高代价或 lethal
- Nav2 规划出来的路径要能引导机器人进入可通行斜坡/楼梯区域

## 当前已完成

当前已经完成的主线能力：

- 新增 `elevation_nav2_bridge::ElevationLayer`，作为 Nav2 costmap layer plugin 加载
- 订阅 `/elevation_mapping_node/elevation_map`
- 支持 GridMap frame 和 costmap frame 不一致时的 TF 转换
- 已验证 local_costmap 可用，典型情况是 local costmap frame 为 `odom`，GridMap frame 为 `map`
- 支持 `cost_source: "elevation" | "traversability" | "fused"`
- `fused` 模式已能读取 cupy 的 `traversability`，并叠加 elevation 邻域台阶高度检查
- 当前四轮足初始能力参数：

```yaml
max_step_height: 0.15
comfortable_step_height: 0.06
max_drop_height: 0.12
step_neighbor_radius: 1
```

- 已有 synthetic 点云测试，可以生成运动本体和移动障碍物
- 当前 bridge 不做全局持久化 buffer，只使用最新 GridMap overlay 到 Nav2 master_grid

## 当前技术定位

当前 `ElevationLayer` 的定位是：

```text
局部 2.5D 地形代价层
```

它更适合放在 `local_costmap`，用于近场地形判断和局部避障。

暂时不把它当作长期全局 2.5D 地图使用。全局规划仍可先依赖普通 2D map / static map / global obstacle 信息。后续如果确实要“走过的地方累计成全局高程地图”，再新增独立 global persistent buffer。

## 研究计划阶段 1：真实 MID360 点云接入

目标：把 synthetic 点云替换成真实 MID360 点云，并保证 frame / timestamp / density 可用。

需要确认：

- MID360 实际 topic 名称，例如 `/livox/lidar` 或 `/livox/points`
- 消息类型是否为 `sensor_msgs/msg/PointCloud2`
- 点云 `header.frame_id` 是什么，例如 `livox_frame` / `mid360` / `livox_lidar`
- TF 是否完整：

```text
map/odom -> base_link -> livox_frame
```

- 点云时间戳是否和 TF buffer 对得上
- elevation_mapping_cupy 的 subscriber 配置是否改成 MID360 topic

需要做的测试：

```bash
ros2 topic info /livox/lidar -v
ros2 topic echo --once /livox/lidar --field header
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo odom base_link
```

阶段验收：

- `/elevation_mapping_node/elevation_map` 正常发布
- `header.frame_id` 符合预期
- RViz 中 elevation map 不漂、不轴反、不跳变
- `ElevationLayer` 日志 `transform_failures=0`

## 研究计划阶段 2：真实点云下的 elevation map 质量调参

目标：让 MID360 点云生成连续、稳定、可用于导航的局部 elevation map。

重点参数：

- `resolution`
- `map_length`
- `min_valid_distance`
- `max_height_range`
- `ramped_height_range_a/b/c`
- `mahalanobis_thresh`
- `wall_num_thresh`
- `enable_visibility_cleanup`
- `enable_overlap_clearance`
- `sensor_noise_factor`

重点问题：

- 地面是否连续
- 机器人自身点云是否被滤掉
- 墙、桌腿、楼梯立面是否过度扩散
- 动态障碍物离开后是否清除
- 斜坡是否被稳定建成连续坡面
- 楼梯是否能保留踏面和立面结构

建议先录包：

```bash
ros2 bag record /livox/lidar /tf /tf_static
```

再离线反复调参数，避免每次都实机跑。

## 研究计划阶段 3：完善 local traversability cost

当前 `fused` 只做了第一版：

```text
final_cost = max(traversability_cost, step_cost)
```

后续要扩展成更适合四轮足的地形代价：

- slope cost：根据 elevation 局部梯度计算坡度
- roughness cost：根据邻域高度残差判断地面粗糙度
- step cost：根据邻域最大高度突变判断台阶/断崖
- unknown policy：区分传感器没看到、被遮挡、确实不可通行
- footprint-aware cost：不只看单个 cell，要看机器人足迹范围内的最大风险

建议输出更多 debug layer：

```text
/local_elevation_height_debug
/local_elevation_traversability_debug
/local_elevation_slope_debug
/local_elevation_step_debug
/local_elevation_fused_debug
```

阶段验收：

- 平地 cost 低
- 普通小坡 cost 低到中
- 陡坡 cost 高
- 低于 15cm 的可跨越台阶不是全部 lethal
- 高于能力上限的台阶/断崖为 lethal

## 研究计划阶段 4：斜坡导航策略

斜坡的核心是：

```text
连续坡面应该可通行，但要根据坡度增加 cost 和降低速度
```

需要做：

- 计算局部坡度角
- 设定四轮足最大安全坡度，例如先从 15 到 25 度范围实验
- 小坡低 cost，中坡中 cost，超过最大坡度 lethal
- 避免把连续坡面当作台阶
- 对下坡和上坡可使用不同阈值

Nav2 侧建议：

- global planner 仍用普通 2D map
- local costmap 加 elevation_layer
- local controller 根据 costmap 选择低风险路径
- 后续可加 speed filter 或 controller 参数切换：坡面区域降低速度

阶段验收：

- 机器人能规划进入可通行斜坡
- 陡坡会绕开
- 斜坡上 costmap 不闪烁
- 路径不会贴着坡边/断崖走

## 研究计划阶段 5：楼梯导航策略

楼梯比斜坡难，因为它不是单纯的 obstacle，也不是单纯的 free space。

四轮足楼梯导航至少要判断：

- 单级高度是否小于能力上限，例如 `<= 0.15m`
- 踏面宽度是否足够落脚/轮足支撑
- 楼梯方向是否和机器人前进方向一致
- 连续台阶是否稳定，而不是随机障碍物
- 下楼梯的 drop 是否安全

当前第一版 `step_height > 0.15m -> lethal` 是保守策略。后续要变成：

```text
低于能力上限的规则台阶：高 cost 但可通行
超过能力上限的高度突变：lethal
没有足够踏面的断崖/杂乱障碍：lethal
```

也就是说，楼梯不能简单依赖单 cell 高度差。需要加入结构判断：

- riser height：立面高度
- tread depth：踏面深度
- stair direction：楼梯主方向
- consecutive steps：是否连续台阶
- landing area：楼梯前后是否有可站立区域

Nav2 侧建议：

- costmap 负责告诉 Nav2 哪些区域可通行/高风险/不可通行
- 真正的上楼梯动作最好交给四轮足底层控制或 gait controller
- Nav2 行为树可在检测到楼梯区域时切换到低速/楼梯模式

阶段验收：

- 可通行楼梯不会被全部打成 lethal
- 超限台阶仍然 lethal
- 杂乱障碍不会被误判成楼梯
- 路径能对准楼梯方向进入，而不是横着切楼梯

## 研究计划阶段 6：真实机器人闭环验证

建议按风险从低到高：

1. 静态平地 MID360 建图
2. 移动平地 local costmap 稳定性
3. 单个低矮障碍物出现/消失
4. 低坡度斜坡
5. 高坡度斜坡
6. 单级 5cm / 10cm / 15cm 台阶
7. 连续两级台阶
8. 真实楼梯低速模式

每一步都记录：

- bag
- RViz 截图
- `ElevationLayer updateCosts` 日志
- robot 是否停下/绕行/通过
- false positive 和 false negative

核心指标：

- `transform_failures=0`
- GridMap 连续性
- local costmap 延迟
- 动态障碍物消失时间
- 斜坡误判率
- 台阶高度判断误差
- 导航成功率

## 当前下一步建议

下一步优先级：

1. 先把当前 synthetic 场景跑稳定：运动本体 + 移动障碍物 + fused local costmap
2. 确认 Jetson 上 YAML 真正生效：

```bash
ros2 param get /local_costmap/local_costmap elevation_layer.cost_source
ros2 param get /local_costmap/local_costmap elevation_layer.max_step_height
ros2 param get /local_costmap/local_costmap elevation_layer.traversability_layer_name
```

3. 接入 MID360 点云，先只看 `/elevation_mapping_node/elevation_map`
4. 再打开 Nav2 local_costmap，确认 bridge 写入 master_grid
5. 最后开始做 slope / stair 结构判断

# 给新对话的接续提示词

请复制下面这段到新 ChatGPT 对话：

```text
你现在继续作为 ROS2 Humble + Nav2 + elevation_mapping_cupy 项目助手。

请先阅读我提供的 project_context.md，不要从零解释 ROS2/Nav2。当前项目已经有：

1. Docker 中运行 elevation_mapping_cupy，发布 /elevation_mapping_node/elevation_map，类型 grid_map_msgs/msg/GridMap。
2. Jetson 宿主机 Nav2 工作空间中新增 package：src/elevation_nav2_bridge。
3. 插件 elevation_nav2_bridge::ElevationLayer 已实现为 Nav2 costmap layer plugin。
4. 插件能订阅 /elevation_mapping_node/elevation_map，并把 GridMap 的 elevation/traversability 转成 Nav2 cost 写入 costmap master_grid。
5. 已有 debug topic：/elevation_traversability_debug，类型 nav_msgs/msg/OccupancyGrid。
6. 当前已实现 TF 变换，可支持 local_costmap 的 odom frame 和 GridMap 的 map/odom frame 不一致。
7. 当前已实现 `cost_source: elevation | traversability | fused`，其中 fused 会读取 cupy traversability，并叠加 elevation 邻域台阶高度检查。
8. 当前目标已经升级为：四轮足机器狗输入 MID360 点云后，通过 2.5D elevation/traversability local costmap，实现斜坡和楼梯场景下的导航。

重要限制：
- 不要把 bridge 放进 Docker。
- 不要修改 navigation2 源码。
- 不要重构整个项目。
- 当前阶段优先围绕 local_costmap 做 2.5D 可通行性调试，不要改 navigation2 源码。
- 当前没有 Gazebo，没有 /clock，因此 use_sim_time 应为 false。
- 后续研究主线是 MID360 点云接入、真实 elevation map 质量调参、slope cost、stair structure 判断、四轮足楼梯/斜坡导航闭环验证。

请基于 project_context.md 继续帮我调试/写代码/分析问题，优先检查：
1. YAML 参数是否真正生效。
2. /global_costmap/costmap 的 info 是否符合 origin/width/height。
3. elevation_layer 的 updateCosts 是否写入 ordinary/lethal cost。
4. GridMap 到 Nav2 costmap 的坐标映射是否正确。
5. `cost_source='fused'`、`traversability_layer_index`、`step_limited` 日志是否符合预期。
6. MID360 点云 frame/timestamp/TF 是否满足 elevation_mapping_cupy 输入要求。
7. 斜坡不要误判成墙，楼梯不要简单全部打成 lethal，要结合四轮足能力判断可通行性。
```
