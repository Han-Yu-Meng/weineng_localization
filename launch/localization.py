#!/usr/bin/env python3
# localization.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import sensor_msgs_py.point_cloud2 as pc2

import os
import sys
import cv2
import yaml
import math
import socket
import argparse
import threading
import subprocess
import numpy as np
import open3d as o3d
from flask import Flask, request, jsonify

import tf2_ros
from tf2_ros import Buffer, TransformListener

class LocalizationOrchestrator(Node):
    def __init__(self, map_name='Weineng'):
        super().__init__('weineng_localization')

        # === 1. 参数声明与路径准备 ===
        self.declare_parameter('min_z', 0.2)
        self.declare_parameter('max_z', 2.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width', 1000)
        self.declare_parameter('height', 1000)
        
        # 声明 map_name 参数，并以传入的 map_name 作为默认值
        self.declare_parameter('map_name', map_name)

        self.map_name = self.get_parameter('map_name').value
        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value
        self.resolution = self.get_parameter('resolution').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        # 定义地图文件路径 ~/Map/{map_name}/map/
        home_dir = os.path.expanduser("~")
        self.map_dir = os.path.join(home_dir, "Map", self.map_name, "map")
        os.makedirs(self.map_dir, exist_ok=True)
        
        self.pcd_path = os.path.join(self.map_dir, "map.pcd")
        self.yaml_path = os.path.join(self.map_dir, "map.yaml")
        self.pgm_path = os.path.join(self.map_dir, "map.pgm")

        self.script_dir = os.path.dirname(os.path.abspath(__file__))

        # === 2. 状态变量与 2D/3D 数据容器 ===
        self.mode = None # "MAPPING" or "LOCALIZATION"
        self.fastlio_proc = None
        self.global_loc_proc = None
        self._intentional_stop = False # 用于标记是否为主动终止进程

        # 核心修改：记录磁盘地图文件最近一次被载入时的时间戳（用于监控热重载）
        self.last_pgm_mtime = 0.0
        self.last_yaml_mtime = 0.0

        # 初始化为 0（代表全图默认已知可行，没有未知区域 -1）
        self.grid = np.zeros((self.height, self.width), dtype=np.int8)
        self.origin_x = -(self.width * self.resolution) / 2.0
        self.origin_y = -(self.height * self.resolution) / 2.0
        
        # 用于累积 3D 点云的列表
        self.accumulated_points = [] 

        # === 3. TF 与 ROS 接口 ===
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cloud_sub = self.create_subscription(PointCloud2, '/cloud_registered', self.cloud_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 10)
        
        # 2D地图使用 Transient Local QoS
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        self.srv_start = self.create_service(Trigger, '/start_mapping', self.ros_start_mapping)
        self.srv_stop = self.create_service(Trigger, '/stop_mapping', self.ros_stop_mapping)

        # 定时器：读取 map->base_link
        self.pose_timer = self.create_timer(0.05, self.publish_pose)
        
        # 定时器：监控子进程健康状态 (每2秒检查一次)
        self.process_monitor_timer = self.create_timer(2.0, self.monitor_processes)

        # 定时器：重定位模式下监控磁盘上的 2D 地图文件修改（每1秒检查一次）
        self.map_watch_timer = self.create_timer(1.0, self.watch_map_files)

        # === 4. HTTP Flask 服务 ===
        self.flask_app = Flask(__name__)
        self.setup_flask_routes()
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()

        # === 5. 初始化逻辑 (判断进入哪种模式) ===
        if os.path.exists(self.pcd_path) and os.path.exists(self.yaml_path):
            self.get_logger().info("=========================================")
            self.get_logger().info(f"检测到地图 {self.map_name} 已存在文件，进入【重定位模式】")
            self.get_logger().info("=========================================")
            self.start_localization_mode()
        else:
            self.get_logger().info("=========================================")
            self.get_logger().info(f"未检测到地图 {self.map_name} 的完整文件，进入【建图模式】")
            self.get_logger().info("=========================================")
            self.start_mapping_mode()

    # ==========================================
    # 核心模式控制逻辑与进程守护
    # ==========================================

    def watch_map_files(self):
        """文件看门狗定时器：监控地图修改"""
        if self.mode != "LOCALIZATION":
            return

        try:
            pgm_changed = False
            yaml_changed = False

            # 校验 pgm 修改时间
            if os.path.exists(self.pgm_path):
                current_pgm_mtime = os.path.getmtime(self.pgm_path)
                if current_pgm_mtime != self.last_pgm_mtime:
                    pgm_changed = True

            # 校验 yaml 修改时间
            if os.path.exists(self.yaml_path):
                current_yaml_mtime = os.path.getmtime(self.yaml_path)
                if current_yaml_mtime != self.last_yaml_mtime:
                    yaml_changed = True

            # 触发重新载入
            if pgm_changed or yaml_changed:
                self.get_logger().warn("检测到磁盘 2D 地图文件（pgm/yaml）发生更改，正在执行热重载并重新发布...")
                self.load_and_publish_2d_map()

        except Exception as e:
            self.get_logger().error(f"看门狗检查地图异常: {e}")

    def monitor_processes(self):
        """守护定时器：检查子进程状态，意外停止则重启"""
        if self._intentional_stop:
            return

        if self.mode == "MAPPING" and self.fastlio_proc:
            if self.fastlio_proc.poll() is not None:
                self.get_logger().error("！！！检测到 FastLIO 意外停止，正在尝试重启 ！！！")
                fastlio_script = os.path.join(self.script_dir, "fastlio.py")
                self.fastlio_proc = subprocess.Popen([sys.executable, fastlio_script])

        elif self.mode == "LOCALIZATION" and self.global_loc_proc:
            if self.global_loc_proc.poll() is not None:
                self.get_logger().error("！！！检测到 Global Localization 意外停止，正在尝试重启 ！！！")
                loc_script = os.path.join(self.script_dir, "global_localization.py")
                self.global_loc_proc = subprocess.Popen([
                    sys.executable, loc_script, "--map", self.map_name
                ])

    def start_mapping_mode(self):
        if self.mode == "MAPPING":
            return True, "已经在建图模式中"
        
        self.stop_all_processes()
        self.mode = "MAPPING"
        
        # 重置地图数据时初始化为全 0（已知可行）
        self.grid = np.zeros((self.height, self.width), dtype=np.int8)
        self.accumulated_points = []

        # 启动 fastlio.py
        fastlio_script = os.path.join(self.script_dir, "fastlio.py")
        self.get_logger().info(f"启动 FastLIO 建图进程: {fastlio_script}")
        self.fastlio_proc = subprocess.Popen([sys.executable, fastlio_script])

        return True, "成功切换至建图模式，正在建图..."

    def stop_mapping_mode(self):
        if self.mode != "MAPPING":
            return True, "当前不在建图模式"
        
        self.get_logger().info("准备停止建图，正在处理地图文件，请稍候...")
        
        # 1. 保存 2D 地图
        self.save_2d_map()

        # 2. 对 3D 点云进行 5cm Voxel Filter 并保存
        self.save_3d_map()

        # 3. 停止 fastlio 进程
        self.stop_all_processes()

        # 4. 切换到重定位模式
        self.get_logger().info("地图保存完毕，即将切换至重定位模式。")
        self.start_localization_mode()

        return True, "建图停止，地图已保存，已切换至重定位模式"

    def start_localization_mode(self):
        self.mode = "LOCALIZATION"
        self.stop_all_processes()

        # 1. 加载并发送 2D 地图 (Transient Local)
        self.load_and_publish_2d_map()

        # 2. 启动 global_localization.py
        loc_script = os.path.join(self.script_dir, "global_localization.py")
        self.get_logger().info(f"启动全局重定位进程: {loc_script}")
        self.global_loc_proc = subprocess.Popen([
            sys.executable, loc_script, "--map", self.map_name
        ])

    def stop_all_processes(self):
        """杀死所有运行的子节点进程"""
        self._intentional_stop = True

        if self.fastlio_proc:
            self.get_logger().info("杀死 FastLIO 进程...")
            try:
                self.fastlio_proc.terminate()
                self.fastlio_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.fastlio_proc.kill()
            self.fastlio_proc = None
            
        if self.global_loc_proc:
            self.get_logger().info("杀死 Global Localization 进程...")
            try:
                self.global_loc_proc.terminate()
                self.global_loc_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.global_loc_proc.kill()
            self.global_loc_proc = None
            
        self._intentional_stop = False

    # ==========================================
    # 数据回调与地图处理
    # ==========================================
    def cloud_callback(self, msg):
        if self.mode != "MAPPING":
            return

        try:
            # 提取点云坐标
            points_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            # 显式转换为标准的 (N, 3) 二维浮点数数组
            points = np.array([[p[0], p[1], p[2]] for p in points_gen], dtype=np.float32)
            
            if len(points) == 0:
                return

            # 1. 累积 3D 点云
            self.accumulated_points.append(points)

            # 2. 生成 2D 栅格地图
            T_odom_cloud = self.tf_buffer.lookup_transform(
                'odom', msg.header.frame_id, msg.header.stamp, rclpy.duration.Duration(seconds=0.1)
            )
            tx = T_odom_cloud.transform.translation.x
            ty = T_odom_cloud.transform.translation.y
            tz = T_odom_cloud.transform.translation.z
            q = T_odom_cloud.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

            sx = int((tx - self.origin_x) / self.resolution)
            sy = int((ty - self.origin_y) / self.resolution)
            
            free_mask = np.zeros_like(self.grid, dtype=np.uint8)
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)

            for p in points:
                lx, ly, lz = p
                gx = lx * cos_y - ly * sin_y + tx
                gy = lx * sin_y + ly * cos_y + ty
                gz = lz + tz
                
                ix = int((gx - self.origin_x) / self.resolution)
                iy = int((gy - self.origin_y) / self.resolution)

                if 0 <= ix < self.width and 0 <= iy < self.height:
                    if self.min_z <= gz <= self.max_z:
                        if 0 <= sx < self.width and 0 <= sy < self.height:
                            cv2.line(free_mask, (sx, sy), (ix, iy), 1, 1)
                        self.grid[iy, ix] = 100
                    elif gz < self.min_z:
                        if 0 <= sx < self.width and 0 <= sy < self.height:
                            cv2.line(free_mask, (sx, sy), (ix, iy), 1, 1)

            # 将非障碍物 (100) 区域全部恢复为已知可行 (0)，消除任何写入 -1 的可能
            self.grid[(free_mask == 1) & (self.grid != 100)] = 0
            
            # 发布 2D 地图
            self.publish_grid_map()

        except Exception as e:
            pass

    def save_3d_map(self):
        if not self.accumulated_points:
            self.get_logger().warn("没有接收到3D点云，无法保存 map.pcd")
            return
            
        self.get_logger().info("正在合并并执行 Voxel Filter (5cm) 处理3D地图...")
        all_points = np.vstack(self.accumulated_points)
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)
        
        downpcd = pcd.voxel_down_sample(voxel_size=0.05)
        o3d.io.write_point_cloud(self.pcd_path, downpcd)
        self.get_logger().info(f"3D地图已保存至: {self.pcd_path}")

    def save_2d_map(self):
        try:
            # 创建全白的背景（255，代表已知可行），不再生成灰色未知区域（205）
            pgm_data = np.full((self.height, self.width), 255, dtype=np.uint8)
            pgm_data[self.grid == 100] = 0 # 障碍物保持为纯黑色 (0)
            cv2.imwrite(self.pgm_path, np.flipud(pgm_data))
            
            with open(self.yaml_path, 'w') as f:
                f.write(f"image: map.pgm\n")
                f.write(f"resolution: {self.resolution}\n")
                f.write(f"origin: [{self.origin_x}, {self.origin_y}, 0.0]\n")
                f.write(f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
            self.get_logger().info(f"2D地图已保存至: {self.pgm_path} 和 .yaml")
        except Exception as e:
            self.get_logger().error(f"保存2D地图失败: {e}")

    def load_and_publish_2d_map(self):
        if not os.path.exists(self.pgm_path) or not os.path.exists(self.yaml_path):
            self.get_logger().error("加载2D地图失败：文件不存在")
            return
        try:
            # 修改：在成功载入数据前更新时间戳，防止触发看门狗再次重载的死循环
            self.last_pgm_mtime = os.path.getmtime(self.pgm_path)
            self.last_yaml_mtime = os.path.getmtime(self.yaml_path)

            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                self.resolution = float(data['resolution'])
                self.origin_x = float(data['origin'][0])
                self.origin_y = float(data['origin'][1])
                
            img = cv2.imread(self.pgm_path, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                self.height, self.width = img.shape
                img = np.flipud(img)
                
                # 重新加载时，默认整张图全为可行区域（0）
                self.grid = np.zeros((self.height, self.width), dtype=np.int8)
                # 图像中凡是偏黑色（小于 127）的像素，全部转为障碍物（100），其余均为可行
                self.grid[img < 127] = 100
                
                self.publish_grid_map()
                self.get_logger().info("加载并重新发布本地2D地图成功。")
        except Exception as e:
            self.get_logger().error(f"加载地图异常: {e}")

    def publish_grid_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(msg)

    def publish_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation
            self.pose_pub.publish(pose)
        except Exception:
            pass

    # ==========================================
    # ROS2 Service 接口
    # ==========================================
    def ros_start_mapping(self, request, response):
        success, msg = self.start_mapping_mode()
        response.success = success
        response.message = msg
        return response

    def ros_stop_mapping(self, request, response):
        success, msg = self.stop_mapping_mode()
        response.success = success
        response.message = msg
        return response

    # ==========================================
    # HTTP Flask 服务接口
    # ==========================================
    def get_ip_address(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

    def setup_flask_routes(self):
        @self.flask_app.route('/start_mapping', methods=['GET'])
        def http_start_mapping():
            success, msg = self.start_mapping_mode()
            return jsonify({"success": success, "message": msg})

        @self.flask_app.route('/stop_mapping', methods=['GET'])
        def http_stop_mapping():
            success, msg = self.stop_mapping_mode()
            return jsonify({"success": success, "message": msg})

        @self.flask_app.route('/upload_map', methods=['POST'])
        def http_set_map():
            if 'pgm' not in request.files or 'yaml' not in request.files:
                return jsonify({"success": False, "message": "Missing files (pgm or yaml)"}), 400
            
            try:
                request.files['pgm'].save(self.pgm_path)
                request.files['yaml'].save(self.yaml_path)
                
                if self.mode == "LOCALIZATION":
                    self.load_and_publish_2d_map()
                    
                return jsonify({"success": True, "message": "Map files updated successfully via HTTP"})
            except Exception as e:
                return jsonify({"success": False, "message": str(e)}), 500

    def run_flask(self):
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        ip = self.get_ip_address()
        print(f"\n[HTTP Server] 控制面板已启动，访问地址: http://{ip}:5000\n")
        self.flask_app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main(args=None):
    # 1. 首先利用 argparse 解析命令行中的自定义 `--map` 参数，过滤掉 ROS2 的特有参数
    parser = argparse.ArgumentParser(description="Weineng Localization orchestrator Node")
    parser.add_argument('--map', type=str, default='Weineng', help='指定地图名称')
    parsed_args, ros_args = parser.parse_known_args()

    # 2. 用剥离了自定义参数后的 ros_args 初始化 ROS2 运行环境
    rclpy.init(args=ros_args)
    
    # 3. 将解析得到的地图名称传入节点初始化函数中
    node = LocalizationOrchestrator(map_name=parsed_args.map)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("程序被用户中断")
    finally:
        node.stop_all_processes()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()