# ROS 2 Foxy / RK3588 构建说明

该配置面向 Ubuntu 20.04、系统 CMake 3.16.3、ROS 2 Foxy、Python 3.8 和
aarch64。PCT、SCAN、
tomography 都在 RK3588 上运行，不要求在 x86 主机预生成 tomogram。

## 设计约束

- tomography 使用 NumPy CPU 后端，不导入 CuPy/CUDA。
- PCD 读取不依赖 Open3D，支持 `DATA ascii` 和 PCL 普通 `DATA binary`。
- `binary_compressed` 地图需要先通过 PCL 转存为普通 binary。
- GTSAM、OSQP 和 pybind11 扩展必须在 RK3588 本机或 aarch64 sysroot 中构建；禁止
  从 x86 工作站复制 `.so`。
- OSQP 1.0 使用仓库维护的 CMake 3.16 兼容补丁；不要为了构建它替换 Ubuntu 系统
  CMake。
- 构建显式关闭 `-march=native`，避免大小核之间产生不可移植指令。
- `src/simulator` 带有 `COLCON_IGNORE`，避免板端编译 Go2、OpenGL、RViz 和仿真地图
  工具；这些组件不是实机 PCT/SCAN 数据链的一部分。

## 板端依赖与构建

```bash
source /opt/ros/foxy/setup.bash
sudo apt update
sudo apt install -y \
  build-essential cmake ninja-build git curl \
  python3-colcon-common-extensions python3-rosdep python3-dev \
  python3-numpy python3-scipy \
  libboost-all-dev libeigen3-dev libopencv-dev libpcl-dev \
  ros-foxy-pcl-conversions ros-foxy-cv-bridge \
  ros-foxy-message-filters ros-foxy-tf2-geometry-msgs \
  ros-foxy-interactive-markers ros-foxy-sensor-msgs-py \
  ros-foxy-rclpy

./scripts/verify_m20pro_baseline.sh --runtime
PCT_BUILD_JOBS=2 ./scripts/setup_pct_dependencies.sh
./scripts/build_workspace.sh
```

在内存紧张或发生编译器 OOM 时将 `PCT_BUILD_JOBS` 改为 `1`。依赖输出目录包含架构
相关二进制，不应提交到 Git，也不能在 x86 与 RK3588 之间共用。

## 在 RK3588 上生成 tomogram

Lightning-LM 保存 `global.pcd` 后，可以直接启动 CPU tomography：

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
export PCT_SCAN_ROS2_ROOT="$PWD"

ros2 launch tomography tomography.launch.py \
  rsg_root:="$PWD/assets/building" \
  scene_name:=building \
  pcd_path:=/absolute/path/to/global.pcd \
  benchmark_repeats:=1
```

输出 pickle 位于 `rsg_root/tomogram/`。运行前应根据 M20Pro 的机身尺寸、楼梯尺寸和
地图分辨率校准 scene 配置中的 `step_max`、`slope_max`、`safe_margin` 与
`inflation`。CPU 生成阶段会占用额外内存；较大地图应先提高 `resolution` 或分区。

## 当前硬件验收边界

源码已消除 Python 3.10、Humble 固定路径、CUDA、Open3D 和 x86 原生指令假设。
最终验收仍必须在真实 M20Pro 上完成，包括 aarch64 全量编译、tomogram 峰值内存、
规划周期、TF/话题对应关系，以及接入运动控制前的限速和急停验证。
