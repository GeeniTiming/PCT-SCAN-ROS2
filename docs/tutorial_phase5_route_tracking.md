# 第五阶段：让 SCAN 严格沿 PCT 路线连续上楼

## 1. 现象不是一个单独的控制器调参问题

RViz 中橙色 `/pct_path` 是 PCT 的全局路线，绿色 `/quad_0/path` 是机器人实际
轨迹。旧实现中绿色轨迹会明显切离橙色路线，而且走完第一个局部段后可能停止。
这里实际包含三个问题：

1. 整条跨层路线经过全局 minimum-snap 拟合后发生严重过冲；
2. 状态机把“局部 B-spline 结束”误当成“全局导航结束”；
3. 平面闭环仿真只更新 x、y、yaw，本身不可能改变 z。

## 2. 全局参考负责守住走廊，局部轨迹负责平滑

使用 Garage 的 2572 点 PCT 路线复现旧全局拟合，测得它相对原折线的最大三维
偏差为 `15.6042 m`，最大水平偏差为 `14.3332 m`。这解释了为什么机器人看起来
像是在跨墙或绕开楼梯：局部规划器拿到的全局参考本身已经离开 PCT 走廊。

当前实现把职责拆开：

```text
PCT 折线 ──RDP(0.10 m)──> 236 个关键点
                              │
                              ├─ 分段线性全局参考：不抄近路
                              │
                              └─ 1.0 m 局部窗口：SCAN B-spline 平滑与避障
```

分段线性并不意味着机器人动作是折线。它只定义全局“导航走廊中心线”；真正发布给
执行器的仍是 SCAN 优化后的三次 B-spline。`planning_horizon:=1.0` 还会把弯道处
单次局部弦线相对原路线的最大偏差控制在约 `0.21 m`。

## 3. 局部完成不等于全局完成

每条 SCAN B-spline 只走大约一个 planning horizon。状态机现在检查全局参考的
`last_progress_time_`：

- 尚未到全局末端：从 `EXEC_TRAJ` 转到 `REPLAN_TRAJ`，继续生成下一窗口；
- 已到全局末端：清除目标并进入 `WAIT_TARGET`。

无界面联调已经连续收到 trajectory 1 到 13，进度从 `1.40/735.91 s` 单调推进到
`13.80/735.91 s`，没有在第一段后停止。

## 4. 为什么 Garage 必须使用 open-loop

这里的 `open_loop` 是演示专用的三维轨迹执行器：它直接采样
`planning/bspline`，发布 x、y、z 和由速度方向得到的 yaw。`closed_loop` 则通过
平面 `cmd_vel` 驱动简化运动学模型，只适合二维演示。

这不代表真实四足机器人应当开环控制。真机链路仍应是：SCAN 期望运动 → 速度或
足端命令 → MoE 低层 policy → 关节执行与状态反馈。本项目当前目标是视频展示，
所以 Garage 联合启动默认使用三维 `open_loop`。

## 5. 十几秒验证一次坡道上升

仓库提供
`src/planner/plan_manage/config/garage_ramp_smoke.yaml`，它截取了固定 PCT 路线的
第一段坡道。先构建并启动：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select scan_planner
source install/setup.bash

ros2 launch scan_planner run.launch.py \
  is_real_world:=false navi_mode:=3 sensor_type:=lidar \
  controller_mode:=open_loop use_gpu:=false \
  use_pcd_map:=true \
  pcd_map_file:="$PWD/assets/garage/pcd/garage_scan_0p20.pcd" \
  reference_path_file:="$PWD/src/planner/plan_manage/config/garage_ramp_smoke.yaml" \
  reference_path_min_distance:=0.2 \
  reference_path_simplify_tolerance:=0.1 \
  planning_horizon:=1.0 \
  map_size_x:=180.0 map_size_y:=240.0 map_size_z:=25.0 \
  init_x:=27.560007095 init_y:=67.617263794 init_z:=0.794042969
```

另一个终端观察机身高度：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /quad_0/body_pose --field pose.pose.position.z
```

本次回归共采样 2277 帧，z 从 `0.794043 m` 连续上升到 `1.309489 m`，净上升
`0.515446 m`；同时连续发布了 11 条局部轨迹。由此可以分别确认“持续滚动规划”
和“执行器真实跟随三维高度”。

## 6. 验收命令

完整 Garage 路线仍用统一脚本：

```bash
./scripts/launch_pct_scan_garage.sh \
  plan_on_startup:=true \
  enable_interactive_markers:=false
```

日志应持续出现递增的 `Received trajectory N`，而不是 trajectory 1 后进入
`WAIT_TARGET`。RViz 中橙色为 PCT 全局路线、绿色为机器人历史轨迹。接下来要做的
是把 Garage SDF 和 Go2 可视模型接入 Gazebo，并由同一个三维 body pose 驱动。
