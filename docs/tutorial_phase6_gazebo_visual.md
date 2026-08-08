# 第六阶段：在 Gazebo 中同步展示 PCT + SCAN 导航

## 1. 本阶段完成了什么

这一阶段把同一条导航结果同时送到两个界面：

```text
RViz Interactive Marker
          │
          ▼
 PCT 跨层 A* ──/pct_path──> SCAN 局部 B-spline
                                  │
                                  ▼
                         /quad_0/body_pose
                           │             │
                           ▼             ▼
                    RViz RobotModel   Gazebo Go2
                                      SetEntityPose
```

RViz 负责算法调试：查看 tomogram、PCT 全局路线、SCAN 局部轨迹和机器人历史轨迹。
Gazebo 负责视频画面：显示完整 Garage SDF，并让 Go2 模型跟随 SCAN 输出的三维机身
位姿。

这不是动力学仿真。项目当前目标是快速得到可解释、可录制的 PCT + SCAN 演示，
因此 Gazebo 不计算接触、摩擦或电机力矩，也不运行 `gz_ros2_control`。这样不会出现
控制器、地面碰撞和低层 policy 同时争夺机身位姿的问题。

## 2. 为什么 SDF、PCD 和 tomogram 可以重合

Garage 的三种表示来自同一个资产坐标系，并保持 identity 变换：

- `assets/garage/upstream/garage/model.sdf`：Gazebo 的三角网格和材质；
- `assets/garage/pcd/garage_scan_0p20.pcd`：SCAN 的碰撞感知地图；
- `assets/garage/tomogram/garage_pct_0p10.pickle`：PCT 的分层可通行图。

因此 PCT 路线点 `(x, y, z)`、SCAN 的 `world` 坐标和 Gazebo 模型坐标无需再做
旋转或平移。PCT 路径中的 z 是地面高度；SCAN 初始化的机身 z 比地面高
`0.4 m`，例如 Garage 起点由 `(16.760, 14.617, 0.090)` 变成机身位姿
`(16.760, 14.617, 0.490)`。

## 3. 代码分工

### 3.1 视觉世界

`src/integration/pct_scan_bridge/worlds/garage_visual.sdf` 加载 Garage 模型、灯光、
默认相机和实体树。世界加载三个最小系统：

- `Physics` 消费外部位姿命令，但不负责生成四足运动；
- `UserCommands` 提供 `/world/garage_visual/set_pose`；
- `SceneBroadcaster` 把模型状态送给 Gazebo GUI。

这里仍会加载最小 Physics system，因为 Gazebo Fortress 需要它消费
`SetEntityPose` 产生的 `WorldPoseCmd`；否则 service 虽然返回成功，模型位姿却不会
改变。Go2 的 xacro 通过 `visual_only:=true` 生成 static 模型，Garage 本身也为
static，因此没有重力下落、接触运动、关节动力学或低层控制。Physics system 在此
只负责把外部位姿命令应用到渲染实体。

### 3.2 ROS 到 Gazebo 的位姿同步

`gazebo_pose_sync.py` 订阅 `/quad_0/body_pose`，以 30 Hz 调用
`ros_gz_interfaces/srv/SetEntityPose`。节点只保留最新一帧位姿，并且同一时刻最多
保留一个未完成请求，避免规划或渲染偶尔变慢时积压数百个 service 请求。

四元数在发送前会归一化；全零四元数会修复为单位四元数。这两种情况都有单元
测试覆盖。

### 3.3 联合启动

`garage_pct_scan.launch.py` 现在包含四部分：PCT bridge、SCAN、RViz 和可选的
Gazebo visual launch。脚本 `launch_pct_scan_gazebo.sh` 把 Gazebo 开关打开，并复用
原有 Garage 脚本中的 PCT 虚拟环境和 GTSAM 动态库设置。

## 4. 构建

在 `PCT-SCAN-ROS2` 根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  go2_description pct_scan_bridge
source install/setup.bash
```

如果前面已经完成过全量构建，这一步只是快速更新本阶段的两个 package。

## 5. 启动交互演示

直接运行：

```bash
./scripts/launch_pct_scan_gazebo.sh
```

脚本会同时打开 RViz 和 Gazebo。在 RViz 中：

1. 选择工具栏的 `Interact`；
2. 保持 Start 在当前机器人附近，拖动 Goal 到希望到达的楼层；
3. 右键 marker，选择 `Plan route`；
4. 观察橙色 PCT 路线、SCAN 局部轨迹和绿色机器人历史轨迹。

Gazebo 中的 `go2_visual` 会同步移动和上楼。需要录制跟随视角时，在右侧
Entity tree 中右键 `go2_visual`，使用 Follow；也可以用鼠标中键平移、滚轮缩放。

当前交互接口表达的是“手动选起终点，由 PCT 搜索跨层路线”。它还不是任意数量的
手绘中间 waypoint。Start 若被拖到远离当前机器人处，SCAN 仍会从当前里程计位置
开始，因此本阶段建议固定 Start、只移动 Goal。多段手绘路线和起点重定位属于后续
交互增强。

## 6. 自动与无界面验收

固定起终点并自动规划：

```bash
./scripts/launch_pct_scan_gazebo.sh \
  plan_on_startup:=true \
  enable_interactive_markers:=false
```

CI 或远程终端可关闭两个 GUI：

```bash
./scripts/launch_pct_scan_gazebo.sh \
  gazebo_headless:=true use_rviz:=false \
  plan_on_startup:=true publish_tomogram:=false \
  enable_interactive_markers:=false
```

成功时应看到以下三类日志：

```text
PCT ... path ...
SCAN ... accepted ...
Received trajectory 1
First Gazebo pose: (16.76, 14.62, 0.49)
Advanced Gazebo pose: (...)
```

`Received trajectory N` 应持续递增；`Advanced Gazebo pose` 表示同一条 SCAN
执行轨迹已经驱动 Gazebo 模型，而不是 Gazebo 中另跑了一套导航。

## 7. 本阶段边界与下一阶段

目前 Go2 的机身位置和朝向是真实的 SCAN 输出，但腿部仍保持 xacro 的静态站姿。
这是视觉链路与导航链路解耦后的预期结果。下一阶段可以加入一个纯视觉 gait
animator：根据机身速度生成 12 关节周期，并同步到 Gazebo；它只改善视频表现，
不会改变 PCT 或 SCAN 的规划结果。
