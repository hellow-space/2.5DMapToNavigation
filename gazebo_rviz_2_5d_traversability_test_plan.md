# Gazebo + RViz 2.5D 可通行性仿真测试方案

## 目标

本文档用于规划和执行 Gazebo + RViz 下的 2.5D 地图可通行性测试。

项目最终目标是：

```text
四轮足机器狗
  -> 输入 MID360 点云
  -> elevation_mapping_cupy 生成 2.5D elevation / traversability GridMap
  -> elevation_nav2_bridge 转成 Nav2 local_costmap 代价
  -> Nav2 结合四轮足运动能力，实现斜坡和楼梯导航
```

Gazebo 仿真的目标不是一开始就完整模拟四轮足动力学，而是先验证感知和导航链路：

```text
Gazebo terrain
  -> simulated MID360 / 3D lidar PointCloud2
  -> elevation_mapping_cupy
  -> /elevation_mapping_node/elevation_map
  -> elevation_nav2_bridge::ElevationLayer
  -> /local_costmap/costmap
  -> RViz / Nav2 behavior
```

核心问题：

- 斜坡是否被识别为连续可通行地形，而不是墙
- 规则楼梯是否能按四轮足能力判断为高代价但可通行
- 超过能力的台阶、断崖、杂乱障碍是否能变成 lethal
- 动态障碍物移走后，costmap 是否能及时消失
- TF、时间戳、GridMap frame 到 costmap frame 的转换是否稳定

## 当前项目基础

当前已经具备：

- `elevation_mapping_cupy` 发布 `/elevation_mapping_node/elevation_map`
- `elevation_nav2_bridge::ElevationLayer` 作为 Nav2 costmap layer plugin
- `ElevationLayer` 支持 TF 转换
- `ElevationLayer` 支持 `cost_source: elevation | traversability | fused`
- local costmap 已能加载 elevation layer
- fused 模式已实现：

```text
final_cost = max(traversability_cost, step_cost)
```

当前四轮足初始能力参数：

```yaml
max_step_height: 0.15
comfortable_step_height: 0.06
max_drop_height: 0.12
step_neighbor_radius: 1
```

## 推荐测试策略

建议分两级仿真。

### 第一级：导航级仿真

先不用完整四轮足动力学模型，只做一个可移动 base_link 和 3D lidar。

目标是验证：

- 点云输入
- elevation map 质量
- traversability / fused cost
- Nav2 local costmap
- RViz 可视化

这一阶段不关心腿部接触、关节力矩、真实 gait，只关心 2.5D 可通行性判断是否正确。

### 第二级：四轮足动力学仿真

在第一级稳定后，再接入四轮足 URDF、ros2_control、gait controller。

目标是验证：

- 斜坡上实际稳定性
- 上下楼梯动作
- Nav2 行为树与 gait mode 切换
- 低速模式、楼梯模式、坡面模式

本文档优先覆盖第一级。

## Gazebo 仿真包建议

建议新增独立包：

```text
src/elevation_gazebo_sim/
  package.xml
  CMakeLists.txt
  launch/
    gazebo_2_5d_nav.launch.py
    gazebo_2_5d_rviz.launch.py
  worlds/
    flat.world
    slope_10deg.world
    slope_20deg.world
    single_step_05cm.world
    single_step_10cm.world
    single_step_15cm.world
    single_step_20cm.world
    stairs_regular.world
    stairs_bad.world
    moving_obstacle.world
  models/
    simple_quadruped_base/
    mid360_lidar/
  config/
    elevation_mapping_mid360_sim.yaml
    nav2_gazebo_local_costmap.yaml
    rviz_2_5d_test.rviz
```

这个包只做仿真资源和 launch，不修改 `navigation2` 源码，不把 `elevation_nav2_bridge` 放进 Docker。

## Gazebo World 设计

每个 world 只验证一个问题，避免多个因素混在一起。

### flat.world

目的：

- 验证点云、TF、elevation map、local costmap 的基础链路
- 验证平地 cost 是否接近 free

期望：

- `/elevation_mapping_node/elevation_map` 连续
- `/local_costmap/costmap` 无异常高代价
- `ElevationLayer updateCosts` 中 `transform_failures=0`

### slope_10deg.world

目的：

- 验证低坡度斜坡不要被误判为 obstacle

期望：

- slope 区域 cost 低到中
- Nav2 能规划进入坡面
- 不应出现整片 lethal

### slope_20deg.world

目的：

- 验证高坡度区域根据机器人能力变成高 cost 或 lethal

期望：

- 如果机器人参数允许 20 度坡，则应是中高 cost
- 如果参数不允许，则应接近 lethal
- 不能出现随机断裂导致路径抖动

### single_step_05cm.world

目的：

- 验证低矮台阶可跨越

期望：

- cost 可升高，但不应 lethal
- `step_limited` 应接近 0

### single_step_10cm.world

目的：

- 验证接近舒适上限但仍可跨越的台阶

期望：

- ordinary cost 增加
- 路径可能偏向平缓区域
- 不应全部打死

### single_step_15cm.world

目的：

- 验证接近当前四轮足能力上限的台阶

期望：

- cost 较高
- 是否 lethal 取决于实际阈值策略
- 如果作为可跨越极限，应保留高代价通路

### single_step_20cm.world

目的：

- 验证超过能力上限的台阶

期望：

- lethal
- `step_limited > 0`
- Nav2 绕行或停止

### stairs_regular.world

目的：

- 验证规则楼梯结构

建议参数：

```text
riser height: 0.10m - 0.15m
tread depth: 0.25m - 0.35m
stair width: >= 1.0m
```

期望：

- 不应把整段楼梯全部当成墙
- 后续 stair structure 判断应能识别连续台阶
- 当前 fused 初版可能偏保守，需要作为调参依据

### stairs_bad.world

目的：

- 验证不规则台阶、断崖、踏面不足时的拒绝能力

期望：

- lethal 或高 cost
- 不应误判成可通行楼梯

### moving_obstacle.world

目的：

- 验证动态障碍物进入和离开

期望：

- 障碍物进入时 local costmap 出现高 cost / lethal
- 障碍物离开后 costmap 能在合理时间内消失
- 如果不消失，优先检查 elevation_mapping_cupy visibility cleanup / overlap clearance / time decay

## 机器人和传感器模型

第一级推荐使用简化机器人：

```text
base_link
  -> livox_frame
```

不需要真实腿部 mesh，也不需要真实 gait。

必须保证 TF：

```text
map -> odom -> base_link -> livox_frame
```

Gazebo 传感器输出：

```text
topic: /livox/lidar 或 /livox/points
type: sensor_msgs/msg/PointCloud2
frame_id: livox_frame
```

第一版可以用普通 3D lidar 近似 MID360，不要求完全复现 Livox 非重复扫描模式。等可通行性链路稳定后，再研究更真实的 MID360 扫描模型。

## ROS 参数原则

Gazebo 环境下所有节点必须统一：

```yaml
use_sim_time: true
```

包括：

- Gazebo bridge
- robot_state_publisher
- elevation_mapping_cupy
- Nav2
- local_costmap
- RViz

如果 `use_sim_time` 不统一，常见现象是：

- TF lookup 失败
- `/elevation_map` 有数据但 costmap 不更新
- RViz 显示延迟或跳变
- `transform_failures` 增加

## elevation_mapping_cupy 仿真配置

建议新增配置：

```text
src/elevation_gazebo_sim/config/elevation_mapping_mid360_sim.yaml
```

核心内容：

```yaml
/elevation_mapping_node:
  ros__parameters:
    use_sim_time: true
    map_frame: "map"
    base_frame: "base_link"
    corrected_map_frame: "map"

    subscribers:
      mid360:
        topic_name: "/livox/lidar"
        data_type: "pointcloud"

    publishers:
      elevation_map:
        layers: ["elevation", "variance", "traversability"]
        basic_layers: ["elevation"]
        fps: 10.0
```

需要重点调：

```yaml
resolution: 0.05 或 0.1
map_length: 6.0 到 10.0
min_valid_distance
max_height_range
ramped_height_range_a
ramped_height_range_b
ramped_height_range_c
sensor_noise_factor
mahalanobis_thresh
enable_visibility_cleanup
enable_overlap_clearance
```

## Nav2 local_costmap 仿真配置

local costmap 推荐：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      track_unknown_space: false
      always_send_full_costmap: true

      plugins: ["elevation_layer"]

      elevation_layer:
        plugin: "elevation_nav2_bridge::ElevationLayer"
        enabled: true
        elevation_topic: "/elevation_mapping_node/elevation_map"
        cost_source: "fused"
        elevation_layer_name: "elevation"
        traversability_layer_name: "traversability"

        free_traversability_threshold: 0.8
        lethal_traversability_threshold: 0.25
        traversability_cost_scale: 120.0

        enable_step_height_check: true
        max_step_height: 0.15
        comfortable_step_height: 0.06
        max_drop_height: 0.12
        step_neighbor_radius: 1

        unknown_as_obstacle: false
        transform_tolerance: 0.2
        publish_debug_grid: true
        debug_grid_topic: "/local_elevation_traversability_debug"
```

后续如果要加普通 obstacle layer，可以放在 elevation layer 之前或之后测试差异，但调 2.5D 原始效果时建议先只开 elevation layer。

## RViz 观察项

RViz Fixed Frame：

```text
map
```

建议显示：

```text
TF
RobotModel
PointCloud2: /livox/lidar
GridMap: /elevation_mapping_node/elevation_map
OccupancyGrid: /local_elevation_traversability_debug
OccupancyGrid: /local_costmap/costmap
Path: /plan
```

观察重点：

- PointCloud2 是否落在机器人周围正确位置
- GridMap 是否连续
- elevation map 是否随机器人运动稳定
- local costmap 是否跟随 rolling window
- fused debug 和 local costmap 是否一致
- 斜坡区域是否高低连续
- 楼梯区域是否保留踏面结构

## 启动顺序建议

推荐分步启动，方便定位问题。

### 1. 启动 Gazebo 和机器人

```bash
ros2 launch elevation_gazebo_sim gazebo_2_5d_nav.launch.py world:=flat
```

检查：

```bash
ros2 topic echo --once /clock
ros2 topic info /livox/lidar -v
ros2 topic echo --once /livox/lidar --field header
ros2 run tf2_ros tf2_echo base_link livox_frame
```

### 2. 启动 elevation_mapping_cupy

```bash
ros2 launch elevation_gazebo_sim elevation_mapping_mid360_sim.launch.py
```

检查：

```bash
ros2 topic echo --once /elevation_mapping_node/elevation_map --field info
ros2 topic echo --once /elevation_mapping_node/elevation_map --field layers
```

期望 layers 包含：

```text
elevation
variance
traversability
```

### 3. 启动 Nav2 local costmap

```bash
ros2 launch elevation_gazebo_sim nav2_local_2_5d.launch.py
```

检查参数：

```bash
ros2 param get /local_costmap/local_costmap elevation_layer.cost_source
ros2 param get /local_costmap/local_costmap elevation_layer.max_step_height
ros2 param get /local_costmap/local_costmap elevation_layer.traversability_layer_name
```

期望：

```text
fused
0.15
traversability
```

### 4. 启动 RViz

```bash
rviz2
```

或：

```bash
ros2 launch elevation_gazebo_sim gazebo_2_5d_rviz.launch.py
```

## 日志判断

重点看 `ElevationLayer` 日志：

```text
ElevationLayer 'elevation_layer' updateCosts:
received_maps=...
cost_source='fused'
elevation_layer_index=...
traversability_layer_index=...
ordinary=...
lethal=...
skipped_unknown=...
out_of_bounds=...
transform_failures=...
step_limited=...
costmap_frame='odom'
gridmap_frame='map'
```

判断：

- `cost_source='fused'`：YAML 生效
- `traversability_layer_index` 有值：GridMap 有 traversability
- `ordinary > 0`：有普通地形代价写入
- `lethal > 0`：有不可通行区域
- `step_limited > 0`：台阶/断崖规则触发
- `transform_failures=0`：TF 正常
- `out_of_bounds` 很大：GridMap 和 costmap 窗口可能没对齐
- `skipped_unknown` 很大：GridMap 有大量 NaN 或无效 cell

## 测试记录模板

每个 world 都记录一行：

```text
world:
date:
commit:
robot model:
lidar topic:
elevation resolution:
local costmap resolution:
cost_source:
max_step_height:
result:
issues:
bag path:
rviz screenshot:
important logs:
```

示例：

```text
world: single_step_10cm
commit: <git commit hash>
cost_source: fused
max_step_height: 0.15
result: pass
issues: step edge cost high but not lethal
important logs: transform_failures=0, step_limited=0, ordinary>0
```

## 验收标准

### 基础链路

- `/livox/lidar` 正常发布 PointCloud2
- TF 链完整
- `/elevation_mapping_node/elevation_map` 连续
- GridMap layers 包含 `elevation` 和 `traversability`
- local costmap 能加载 `ElevationLayer`
- `transform_failures=0`

### 平地

- 平地 cost 接近 free
- 没有大面积 lethal
- 地图不闪烁

### 斜坡

- 小坡可通行
- 陡坡高 cost 或 lethal
- 斜坡不应被当成墙
- 路径能沿坡面方向规划

### 台阶

- 5cm / 10cm 台阶不应全部 lethal
- 15cm 台阶接近能力边界，应高 cost
- 20cm 台阶应 lethal
- `step_limited` 对超限高度有响应

### 楼梯

- 规则楼梯不能简单整段 lethal
- 超限楼梯或断崖应 lethal
- 后续 stair structure 判断应能识别踏面、立面和方向

### 动态障碍物

- 进入时 cost 出现
- 离开时 cost 消失
- 消失时间可接受

## 常见问题

### 有点云但没有 elevation map

检查：

```bash
ros2 topic echo --once /livox/lidar --field header
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 param get /elevation_mapping_node subscribers.mid360.topic_name
```

常见原因：

- topic 名称不对
- frame_id 没有 TF
- `use_sim_time` 不一致
- 点云高度范围被过滤

### elevation map 有断裂

可能原因：

- 点云太稀
- lidar 扫描模型不覆盖地面
- resolution 太高
- visibility cleanup 太 aggressive
- 机器人自身遮挡

处理：

- 降低 elevation map resolution
- 增加点云密度
- 调整 lidar 俯仰角
- 调整 `min_valid_distance` / `max_height_range`
- 暂时关闭部分 cleanup 进行对照

### costmap 不更新

检查：

```bash
ros2 topic echo --once /local_costmap/costmap --field header
ros2 param get /local_costmap/local_costmap use_sim_time
ros2 run tf2_ros tf2_echo odom base_link
```

常见原因：

- Nav2 没 lifecycle active
- costmap frame 和 GridMap frame 缺 TF
- `use_sim_time` 不一致
- `ElevationLayer` 没加载

### 楼梯全部 lethal

当前 fused 初版偏保守，原因可能是：

```text
step_up > max_step_height -> lethal
```

后续需要 stair structure 判断，把规则台阶从“障碍物”升级为“高代价但可通行地形”。

## 后续扩展

建议后续在 `ElevationLayer` 中增加 debug 输出：

```text
/local_elevation_height_debug
/local_elevation_traversability_debug
/local_elevation_slope_debug
/local_elevation_step_debug
/local_elevation_fused_debug
```

当前 traversability 在 Nav2 bridge 中只作为 soft risk，不应单独产生 lethal。`lethal_traversability_threshold` 是历史参数名，实际语义更接近 full-risk threshold。明确的物理硬限制应来自 slope、step、drop、stair structure 等可解释指标。

建议后续增加 cost components：

```text
slope cost
roughness cost
step height cost
stair structure cost
unknown / occlusion cost
footprint-aware cost
```

最终目标：

```text
普通平地: free / low cost
小坡: low to medium cost
陡坡: high cost or lethal
规则可通行楼梯: high cost but passable
超限台阶/断崖/杂乱障碍: lethal
```
