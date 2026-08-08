# 第三阶段：生成 Garage PCT tomogram

## 1. 为什么不用 Building 的 0.10 m 配置

Garage 点云范围约为 128 × 143 × 21 m。若直接使用 0.10 m 栅格和 0.50 m
高度切片，初始三维地图约有 7,718 万个单元。PCT 会同时维护多份 float32 GPU
数组，并在梯度、通行性和层简化时创建临时数组，8 GB 显卡很容易显存不足。

本场景采用：

```text
XY resolution = 0.20 m
ground height = -0.20 m
slice height  = 0.50 m
kernel size   = 5 cells
slope max     = 0.40 rad
step max      = 0.20 m
safe margin   = 0.40 m
inflation     = 0.20 m
```

PCD 仍使用 0.10 m 版本。点云采样密度和 tomogram 栅格分辨率是两个不同参数：
较密的输入点有助于保留地面与坡道观测，PCT 再把它们投影到 0.20 m XY 栅格。

## 2. SCAN 仓库内的离线生成器

运行：

在 `PCT-SCAN-ROS2` 仓库根目录运行：

```bash
./scripts/generate_garage_tomogram.sh
```

脚本使用本仓库 `src/pct/tomography` 和 `.venv-tomography`。它不会启动 ROS 或
RViz。输出位置是：

```text
assets/garage/tomogram/garage_pct_0p10.pickle
```

如果输出已经存在，脚本默认停止。确认需要重新生成时使用：

```bash
./scripts/generate_garage_tomogram.sh --force
```

## 3. 两个兼容性处理

PCT 上游 tomography 节点会为了论文性能统计运行一次预热和十次计时。离线生成
资产不需要重复计算，本工具只运行一次 `point2map()`。

上游 CUDA helper 把坐标参数声明成未定义的 `float16`，但点云、NumPy 数组和 GPU
buffer 实际都是 float32。本工具只在当前进程中把 helper 的坐标类型替换成 CUDA
`float`，不修改第三方 checkout。

CUDA/CuPy 可用时生成器走原 PCT GPU 路径；驱动不可用时会自动回退到
NumPy/SciPy CPU 实现，不再依赖外部 PCT 虚拟环境。

生成进程使用的 NumPy 2.x pickle 不能直接由 ROS 2 Humble 的系统 NumPy 1.21
读取。生成器最后会调用干净的 `/usr/bin/python3` 重写一次 pickle，使 ROS 2 节点
可以直接反序列化；这个步骤只改变序列化兼容性，不改变数组数值。

## 4. 本机生成结果

```text
input points:     4,517,125
initial grid:     644 × 718 × 43
initial cells:    19,882,856
simplified shape: 14 × 644 × 718
finite cells:     3,378,983
free cells:       2,549,740  (A* cost < 50 during generation summary)
elapsed:          6.91 s
pickle size:      61.7 MiB
```

pickle 中的 `data` 形状为 `(5, 14, 644, 718)`，五个通道依次是通行代价、X
方向代价梯度、Y 方向代价梯度、地面高度和顶面高度。

## 5. 验收

使用 PCT A* 的实际阈值 20 检查：

```bash
.venv-tomography/bin/python \
  scripts/inspect_pct_tomogram.py
/usr/bin/python3 -c \
  'import pickle; pickle.load(open("assets/garage/tomogram/garage_pct_0p10.pickle", "rb")); print("system pickle load: PASS")'
```

检查器会验证键名、数组形状、每层可通行单元、最大二维连通分量，以及相邻层之间
的 gateway 候选。最后应看到：

```text
[inspect] PASS: structure, traversable cells and adjacent gateways are valid
```

gateway 存在说明数据具备换层条件，但它还不是任意起终点必定可达的证明。完整验收
仍需在下一阶段用 PCT 原生 A* 规划一条从底层到顶层的路线。

## 6. 下一阶段必须处理的动态层数

Garage 被简化为 14 层，而当前 ROS 2 PCT wrapper 在初始化 C++ planner 时把层数
硬编码成了 15。正确做法是传入 pickle 的 `self.n_slice`，而不是添加一层伪数据。
该问题已经由仓库内的 `pct_scan_bridge` 解决，下一步见
[`tutorial_phase4_pct_scan_bridge.md`](tutorial_phase4_pct_scan_bridge.md)。
