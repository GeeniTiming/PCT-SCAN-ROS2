# 第四阶段：打通 Garage 的 PCT 全局规划与 SCAN 局部规划

这一阶段完成的是完整导航链路中的算法接口：PCT 在 tomogram 上搜索跨楼层全局
路线，并通过 `/pct_path` 发布 `nav_msgs/msg/Path`；SCAN 在 Mode 3 中接收该路线，
生成严格沿 PCT 折线的全局参考轨迹，再滚动生成局部 B-spline。

```text
Garage pickle
    ↓
PCT native A*（跨 tomogram layer）
    ↓  /pct_path: nav_msgs/Path
三维降采样 + RDP + 分段线性全局参考
    ↓
SCAN 滚动局部 B-spline
    ↓  planning/bspline
三维 open-loop 演示执行器
```

## 1. 为什么需要 `pct_scan_bridge`

Garage pickle 的真实形状是 `(5, 14, 644, 718)`，即 14 个 tomogram layer。
第三方 ROS 2 wrapper 原先把 native planner 的层数固定为 15，这会让 C++ A* 对
错误的内存布局建图。`pct_scan_bridge` 从 pickle 读取 `self.n_slice`，把真实层数
传给 native planner；第三方 checkout 保持只读。

需要注意，`OfflineElePlanner.init_map()` 参数中的第二个 `15` 是步长代价权重，
第四个参数才是层数。桥接器只替换第四个参数。

## 2. 先做无 ROS 图形界面的 A* 验收

在仓库根目录构建桥接包：

```bash
source /opt/ros/humble/setup.bash
./scripts/build_workspace.sh --packages-select pct_scan_bridge scan_planner
```

然后运行：

```bash
./scripts/test_pct_garage_astar.sh --force
```

固定验收路线从一层上升至五层：

```text
start = (16.760008, 14.617265,  0.089722), layer 0
goal  = (41.360008, 35.017265, 13.890625), layer 9
```

成功结果应包含 2572 个 ROS 路径点、native layer 0 到 9，以及约 13.8 m 的高度
增量。CSV 验收结果写入 `assets/garage/routes/f1_to_f5.csv`。

这里不能只按“目标点看起来在顶层”来选终点。点属于可通行栅格，并不代表它和
底层通过 gateway 连通；之前的顶层候选点就在另一个连通分量中，所以 A* 会正确
返回失败。现在的默认终点属于与坡道 gateway 相连的顶层主分量。

## 3. 为什么不能把 2572 个点直接交给 SCAN

PCT 的输出是 0.20 m 栅格上的 A* cell 序列。直接保留全部 cell 没有必要，也会
增加后续投影、显示和局部目标采样的开销。

联合启动先按三维距离 `0.20 m` 去重，再执行三维
Ramer–Douglas–Peucker（RDP）简化。最大几何偏差设为 `0.10 m`，本路线从
2572 个原始姿态变为 236 个折线路点。它和“每隔 N 个点取一个”不同：RDP 会
自动保留坡道转折和明显曲率，并给出可解释的误差上界。

相关启动参数是：

```text
reference_path_min_distance:=0.2
reference_path_simplify_tolerance:=0.1
```

这里还有一个很重要的工程修正：不能再对整条多楼层路线做一条全局
minimum-snap 拟合。实测旧拟合相对 PCT 折线的最大三维偏差达到 `15.60 m`，会在
弯道和楼层连接处抄近路。现在的全局轨迹采用恒速分段线性参数化，严格经过每一段
PCT 折线；平滑和避障仍由 SCAN 的短距离局部 B-spline 完成。

## 4. 一条命令启动 PCT + SCAN

```bash
./scripts/launch_pct_scan_garage.sh
```

脚本自动完成以下工作：

- source ROS 2 Humble、PCT overlay 和本工程 overlay；
- 补充 GTSAM、OSQP 与 cJSON 的运行库路径；
- 加载 Garage PCD 和 tomogram；
- 启动 PCT 交互路点、SCAN Mode 3、运动学机器人和 RViz。

在 RViz 中选择 `Interact`，拖动 `Start` 和 `Goal` 球体。右键任意球体并选择
`Plan route` 后，PCT 发布新路径，SCAN 会立即更新全局参考轨迹。

验证固定起终点且不打开 RViz：

```bash
./scripts/launch_pct_scan_garage.sh \
  use_rviz:=false \
  plan_on_startup:=true \
  publish_tomogram:=false \
  enable_interactive_markers:=false
```

关键成功日志为：

```text
Trajectory with 2572 poses published on /pct_path
Reference path reduced from 2572 poses to 236 trajectory waypoints
Reference path accepted
Received trajectory 1
Received trajectory 2
Received trajectory 3
```

`/pct_path` 两端使用 `reliable + transient_local + depth 1`。因此即使 SCAN 稍晚
完成初始化，也能取得 PCT 保存的最近一条路径，无需重复运行 A*。

## 5. 三维跟踪方式与下一阶段

`closed_loop` 的 `go2_kinematic_sim` 接收平面 `cmd_vel`，只积分 x、y 和 yaw，
因此它不能用于跨层视频。Garage 联合启动默认改为 `open_loop`：执行器直接采样
SCAN 输出的三维 B-spline，机身 z 会随坡道上升。

SCAN 的每条局部 B-spline 只覆盖一个 planning horizon。状态机必须在局部段结束时
继续规划下一段，直到全局 PCT 终点；否则会走约一米后错误回到 `WAIT_TARGET`。
该修复及坡道验收见
[`tutorial_phase5_route_tracking.md`](tutorial_phase5_route_tracking.md)。下一阶段是在
Gazebo 中加载 Garage SDF，并用 `/quad_0/body_pose` 驱动 Go2 可视模型。这个执行器
只负责视频演示，不伪装成真实动力学或低层 MoE policy。
