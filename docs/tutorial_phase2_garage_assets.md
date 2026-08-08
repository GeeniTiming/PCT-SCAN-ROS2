# 第二阶段：准备 Garage 点云资产

## 1. 本步骤的目标

Garage 场景提供了两类互相配准的资产：

- `meshes/garage.dae`：Gazebo 用于渲染建筑细节；
- `preview/pointcloud.ply`：PCT 和 SCAN 使用的几何点云。

转换过程只做体素降采样和文件格式转换，不施加旋转、平移或缩放。因此生成的
PCD 与 Gazebo 网格继续共享同一个世界坐标系。

## 2. 为什么生成两个 PCD

PCT 的现有 Building 配置使用 0.10 m tomogram 栅格。跨楼层规划对坡道边缘和楼层
连接较敏感，所以为 PCT 保留一份 0.10 m 点云。RViz 显示和 SCAN 局部地图更重视
实时性，可以使用 0.20 m 轻量点云：

| 文件 | 体素尺寸 | 用途 |
| --- | ---: | --- |
| `garage_pct_0p10.pcd` | 0.10 m | 生成 PCT tomogram |
| `garage_scan_0p20.pcd` | 0.20 m | SCAN 静态地图和 RViz 显示 |

体素降采样把空间划分为固定大小的立方体，并用每个非空体素内点的质心代表该
体素。它降低点数，但不会改变坐标系。体素过大会把坡道、栏杆和楼板边缘合并，
并可能破坏跨层 A* 所需的连通区域。

## 3. 转换命令

本项目的转换脚本依赖 Open3D，先创建仓库内的生成环境：

```bash
# 在 PCT-SCAN-ROS2 仓库根目录
PLY="$PWD/assets/garage/upstream/garage/preview/pointcloud.ply"
PCD_DIR="$PWD/assets/garage/pcd"
./scripts/setup_tomography_env.sh

.venv-tomography/bin/python \
  scripts/convert_pointcloud.py \
  "$PLY" "$PCD_DIR/garage_pct_0p10.pcd" \
  --voxel-size 0.10

.venv-tomography/bin/python \
  scripts/convert_pointcloud.py \
  "$PLY" "$PCD_DIR/garage_scan_0p20.pcd" \
  --voxel-size 0.20
```

脚本默认拒绝覆盖已有文件。如果确认需要重新生成，才添加 `--force`。

## 4. 本机实测结果

原始 PLY 包含 4,617,947 个点，边界为：

```text
min = [-40.180378, -25.367056, -0.111863]
max = [ 87.700394, 117.401588, 20.902609]
```

转换结果：

```text
garage_pct_0p10.pcd:  4,517,125 points, 51.7 MiB
garage_scan_0p20.pcd: 2,214,297 points, 25.3 MiB
```

0.10 m 版本只减少约 2.2% 的点，说明上游点云本身已接近 10 cm 采样。0.20 m
版本减少约 52.1%，适合实时显示。

## 5. 验收

检查 PCD 头：

```bash
head -n 11 assets/garage/pcd/garage_pct_0p10.pcd
```

关键字段应为：

```text
FIELDS x y z
WIDTH 4517125
POINTS 4517125
DATA binary
```

输出边界与输入边界只应有小于一个体素的差异。如果出现数十米平移、坐标轴交换
或整体缩放，则不能继续生成 tomogram，必须先检查转换过程是否错误地加入了变换。

## 6. 下一步

下一步使用 SCAN 仓库内的离线生成器，从 `garage_pct_0p10.pcd` 生成 tomogram。
生成器设置地图高度范围、切片间距、最大坡度和安全膨胀距离，并检查五层楼面是否
通过连续坡道形成可搜索的相邻层 gateway。
