#!/usr/bin/python3
"""CPU PCT tomography node for ROS 2 Foxy/Humble and ARM64."""

# import argparse
import os
import time
import pickle
import numpy as np
import importlib

  
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


from .tomogram import Tomogram
from .pcd_io import read_point_cloud_xyz

# equal sys.path.append("../")
# parent_dir = pathlib.Path(__file__).resolve().parent
# if str(parent_dir) not in sys.path:
#    sys.path.append(str(parent_dir))

from .config import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from .config import Config
from .config import scene

# rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class Tomography(Node):
    def __init__(self, cfg: Config):
        super().__init__('pointcloud_tomography')

        self.declare_parameter("rsg_root", "")
        rsg_root_param = self.get_parameter("rsg_root")
        self.declare_parameter("scene_name", "")
        scene_name_param = self.get_parameter("scene_name")
        self.declare_parameter("pcd_path", "")
        self.declare_parameter("benchmark_repeats", 1)

        if not rsg_root_param.value:
            raise ValueError("Missing required parameter: rsg_root")
        
        if not scene_name_param.value:
            raise ValueError("Missing required parameter: scene_name")
        
        self.rsg_root = rsg_root_param.get_parameter_value().string_value
        self.scene_name = scene_name_param.get_parameter_value().string_value.lower()
        
        # scene_module_name = f"config.scene_{self.scene_name}"
        # Use relative import for dynamic loading or absolute package path
        # Assuming 'tomography' is the package name
        scene_module_name = f"tomography.config.scene_{self.scene_name}"
        scene_class_name = f"Scene{self.scene_name.capitalize()}"

        try:
            scene_module = importlib.import_module(scene_module_name)
        except ModuleNotFoundError:
             # Fallback to relative import if running as script or different structure
             scene_module = importlib.import_module(f".config.scene_{self.scene_name}", package="tomography")

        scene_cfg: scene.Scene = getattr(scene_module, scene_class_name)()

        self.cfg = cfg

        self.qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.export_dir = os.path.join(
            self.rsg_root, cfg.map.export_dir.strip("/"))
        pcd_path = self.get_parameter("pcd_path").value
        self.pcd_file = pcd_path if pcd_path else scene_cfg.pcd.file_name
        self.benchmark_repeats = max(
            1, int(self.get_parameter("benchmark_repeats").value))
        self.resolution = scene_cfg.map.resolution
        self.ground_h = scene_cfg.map.ground_h
        self.slice_dh = scene_cfg.map.slice_dh

        self.center = np.zeros(2, dtype=np.float32)
        self.tomogram = Tomogram(scene_cfg)

        self.get_logger().info(f"PCD file name: {self.pcd_file}")
        if self.pcd_file is None:
            raise ValueError("PCD file name is not specified.")
        else:
            points = self.loadPCD()

        # Process
        self.process(points)

    def initROS(self):
        self.map_frame = self.cfg.ros.map_frame
        pointcloud_topic = self.cfg.ros.pointcloud_topic
        layer_G_topic = self.cfg.ros.layer_G_topic
        layer_C_topic = self.cfg.ros.layer_C_topic
        tomogram_topic = self.cfg.ros.tomogram_topic


        self.pointcloud_pub = self.create_publisher(PointCloud2, pointcloud_topic, self.qos)

        self.layer_G_pub_list = []
        self.layer_C_pub_list = []

        for i in range(self.n_slice):
            layer_G_pub = self.create_publisher(PointCloud2, layer_G_topic + str(i), self.qos)
            self.layer_G_pub_list.append(layer_G_pub)
            layer_C_pub = self.create_publisher(PointCloud2, layer_C_topic + str(i), self.qos)
            self.layer_C_pub_list.append(layer_C_pub)

        self.tomogram_pub = self.create_publisher(PointCloud2, tomogram_topic, self.qos)

    def loadPCD(self):
        pcd_path = self.pcd_file
        if not os.path.isabs(pcd_path):
            pcd_path = os.path.join(self.rsg_root, "pcd", pcd_path)
        points = read_point_cloud_xyz(pcd_path)

        self.get_logger().info(f"PCD points: {points.shape[0]}")

        self.points_max = np.max(points, axis=0)
        self.points_min = np.min(points, axis=0)           
        self.points_min[-1] = self.ground_h
        self.map_dim_x = int(np.ceil((self.points_max[0] - self.points_min[0]) / self.resolution)) + 4
        self.map_dim_y = int(np.ceil((self.points_max[1] - self.points_min[1]) / self.resolution)) + 4
        n_slice_init = int(np.ceil((self.points_max[2] - self.points_min[2]) / self.slice_dh))
        self.center = (self.points_max[:2] + self.points_min[:2]) / 2
        self.slice_h0 = self.points_min[-1] + self.slice_dh
        self.tomogram.initMappingEnv(self.center, self.map_dim_x, self.map_dim_y, n_slice_init, self.slice_h0)

        self.get_logger().info(f"Map center: [{self.center[0]:.2f}, {self.center[1]:.2f}]", )
        self.get_logger().info(f"Dim_x: {self.map_dim_x}")
        self.get_logger().info(f"Dim_y: {self.map_dim_y}")
        self.get_logger().info(f"Num slices init: {n_slice_init}")

        self.VISPROTO_I, self.VISPROTO_P = \
            GRID_POINTS_XYZI(self.resolution, self.map_dim_x, self.map_dim_y)

        return points
        
    def process(self, points):        
        t_map = 0.0
        t_trav = 0.0
        t_simp = 0.0
        t_all = 0.0
        n_repeat = self.benchmark_repeats

        # One warm-up pass plus an optional small number of benchmark passes.
        # The default is one to keep startup cost bounded on RK3588.
        for i in range(n_repeat + 1):
            t_start = time.time()
            layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, t_gpu = self.tomogram.point2map(points)

            if i > 0:
                t_map += t_gpu['t_map']
                t_trav += t_gpu['t_trav']
                t_simp += t_gpu['t_simp']
                t_all += (time.time() - t_start) * 1e3

        self.get_logger().info(f"Num slices simp: {layers_g.shape[0]}")
        self.get_logger().info(f"Num repeats (for benchmarking only): {n_repeat}")
        self.get_logger().info(f" -- avg t_map  (ms): {t_map / n_repeat}")
        self.get_logger().info(f" -- avg t_trav (ms): {t_trav / n_repeat}")
        self.get_logger().info(f" -- avg t_simp (ms): {t_simp / n_repeat}")
        self.get_logger().info(f" -- avg t_all  (ms): {t_all / n_repeat}")

        self.n_slice = layers_g.shape[0]
        
        map_file = os.path.splitext(os.path.basename(self.pcd_file))[0]

        self.exportTomogram(np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c)), map_file)

        self.initROS()
        self.publishPoints(points)
        self.publishLayers(self.layer_G_pub_list, layers_g, layers_t)
        self.publishLayers(self.layer_C_pub_list, layers_c, None)
        self.publishTomogram(layers_g, layers_t)

    def exportTomogram(self, tomogram, map_file):        
        data_dict = {
            'data': tomogram.astype(np.float16),
            'resolution': self.resolution,
            'center': self.center,
            'slice_h0': self.slice_h0,
            'slice_dh': self.slice_dh,
        }
        file_name = map_file + '.pickle'
        os.makedirs(self.export_dir, exist_ok=True)
        output_path = os.path.join(self.export_dir, file_name)
        with open(output_path, 'wb') as handle:
            pickle.dump(data_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

        self.get_logger().info(f"Tomogram exported: {output_path}")

    def publishPoints(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()

        header.frame_id = self.map_frame

        point_msg = pc2.create_cloud_xyz32(header, points)
        self.pointcloud_pub.publish(point_msg)

    def publishLayers(self, pub_list, layers, color=None):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()

        header.frame_id = self.map_frame

        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.center

        for i in range(layers.shape[0]):
            layer_points[:, 2] = layers[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            if color is not None:
                layer_points[:, 3] = color[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            else:
                layer_points[:, 3] = 1.0
        
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, valid_points)
            pub_list[i].publish(points_msg) 

    def publishTomogram(self, layers_g, layers_t):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        # Extract grid points for all layers at once
        idx_x = self.VISPROTO_I[:, 0]
        idx_y = self.VISPROTO_I[:, 1]
        
        flat_g = layers_g[:, idx_x, idx_y].copy()
        flat_t = layers_t[:, idx_x, idx_y].copy()
        
        n_slice = flat_g.shape[0]

        # Apply tomogram visibility logic
        for i in range(n_slice - 1):
            diff = flat_g[i + 1] - flat_g[i]
            mask_h = diff < self.slice_dh
            flat_g[i, mask_h] = np.nan
            flat_t[i + 1, mask_h] = np.minimum(flat_t[i, mask_h], flat_t[i + 1, mask_h])

        # Flatten arrays to list of points
        g_all = flat_g.flatten()
        t_all = flat_t.flatten()
        
        # Create corresponding XY coordinates
        base_xy = self.VISPROTO_P[:, :2] + self.center
        xy_all = np.tile(base_xy, (n_slice, 1))

        # Filter valid points
        valid_mask = ~np.isnan(g_all)
        
        if np.any(valid_mask):
            global_points = np.column_stack((
                xy_all[valid_mask],
                g_all[valid_mask],
                t_all[valid_mask]
            )).astype(np.float32)
        else:
            global_points = np.empty((0, 4), dtype=np.float32)

        points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, global_points)
        self.tomogram_pub.publish(points_msg)


def main(args=None):
    rclpy.init(args=args)
    
    cfg = Config()    
    node = Tomography(cfg)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # ROS 2 launch may already have shut the context down after SIGINT.
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
