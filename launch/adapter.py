#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import numpy as np
import tf2_ros
from tf2_ros import Buffer, TransformListener
import sensor_msgs_py.point_cloud2 as pc2
import os
import cv2
import threading
import math
import yaml
from flask import Flask, request, jsonify

# 尝试导入自定义服务，如果不存在则定义备用类
try:
    from finenav_msgs.srv import SetMap
except ImportError:
    class SetMap:
        class Request:
            def __init__(self): self.map = OccupancyGrid()
        class Response:
            def __init__(self): self.success = False; self.message = ""

class CloudToMapAdapter(Node):
    def __init__(self):
        super().__init__('cloud_to_map_adapter')

        # 声明参数（ROS 2 会自动根据 Launch 文件覆盖这些值）
        self.declare_parameter('min_z', 0.2)
        self.declare_parameter('max_z', 2.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width', 1000)
        self.declare_parameter('height', 1000)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value
        self.resolution = self.get_parameter('resolution').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # 地图初始化 (-1: 未知, 100: 占据, 0: 空闲)
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self.origin_x = -(self.width * self.resolution) / 2.0
        self.origin_y = -(self.height * self.resolution) / 2.0

        self.is_mapping = False
        self.has_disk_map = False
        self.initial_pose_in_odom = None 

        # TF 监听器（自动关联节点时钟）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 发布者与订阅者
        self.cloud_sub = self.create_subscription(PointCloud2, '/cloud_registered', self.cloud_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/localization_pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # 服务
        self.start_srv = self.create_service(Trigger, '/start_mapping', self.start_mapping_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_mapping', self.stop_mapping_callback)
        
        try:
            from finenav_msgs.srv import SetMap as ActualSetMap
            self.set_map_srv = self.create_service(ActualSetMap, '/set_map', self.set_map_callback)
        except ImportError:
            self.get_logger().error("finenav_msgs.srv.SetMap not found.")

        # 定时器：Pose 发布频率高些，Map 发布频率低些
        self.timer = self.create_timer(0.05, self.publish_pose)
        self.map_timer = self.create_timer(1.0, self.publish_map)

        # 加载本地已有地图
        self.load_map()

        # Flask HTTP 服务
        self.flask_app = Flask(__name__)
        self.setup_flask_routes()
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()

        self.get_logger().info("Cloud to Map Adapter with Correct TimeSync Initialized.")

    def setup_flask_routes(self):
        @self.flask_app.route('/upload_map', methods=['POST'])
        def upload_map():
            if 'pgm' not in request.files or 'yaml' not in request.files:
                return jsonify({"success": False, "message": "Missing files"}), 400
            
            dir_path = os.path.dirname(os.path.realpath(__file__))
            try:
                request.files['pgm'].save(os.path.join(dir_path, "map.pgm"))
                request.files['yaml'].save(os.path.join(dir_path, "map.yaml"))
                self.load_map()
                self.publish_map()
                return jsonify({"success": True, "message": "Map updated via HTTP"})
            except Exception as e:
                return jsonify({"success": False, "message": str(e)}), 500

    def run_flask(self):
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        self.flask_app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

    def cloud_callback(self, msg):
        if not self.is_mapping:
            return

        try:
            # 关键：查询点云产生那一时刻的位姿
            T_odom_cloud = self.tf_buffer.lookup_transform(
                self.odom_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.1)
            )
            
            # 解析变换
            tx_oc = T_odom_cloud.transform.translation.x
            ty_oc = T_odom_cloud.transform.translation.y
            q_oc = T_odom_cloud.transform.rotation
            yaw_oc = math.atan2(2 * (q_oc.w * q_oc.z + q_oc.x * q_oc.y), 1 - 2 * (q_oc.y * q_oc.y + q_oc.z * q_oc.z))

            if self.initial_pose_in_odom:
                # 转换到起始建图坐标系
                tx_oi = self.initial_pose_in_odom.transform.translation.x
                ty_oi = self.initial_pose_in_odom.transform.translation.y
                q_oi = self.initial_pose_in_odom.transform.rotation
                yaw_oi = math.atan2(2 * (q_oi.w * q_oi.z + q_oi.x * q_oi.y), 1 - 2 * (q_oi.y * q_oi.y + q_oi.z * q_oi.z))
                
                dx, dy = tx_oc - tx_oi, ty_oc - ty_oi
                tx = dx * math.cos(yaw_oi) + dy * math.sin(yaw_oi)
                ty = -dx * math.sin(yaw_oi) + dy * math.cos(yaw_oi)
                tz = T_odom_cloud.transform.translation.z - self.initial_pose_in_odom.transform.translation.z
                yaw = yaw_oc - yaw_oi
            else:
                tx, ty, tz, yaw = tx_oc, ty_oc, T_odom_cloud.transform.translation.z, yaw_oc

            # 处理点云数据
            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            sx, sy = int((tx - self.origin_x) / self.resolution), int((ty - self.origin_y) / self.resolution)
            
            free_mask = np.zeros_like(self.grid, dtype=np.uint8)
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)

            for p in points:
                lx, ly, lz = p
                gx = lx * cos_y - ly * sin_y + tx
                gy = lx * sin_y + ly * cos_y + ty
                gz = lz + tz
                ix, iy = int((gx - self.origin_x) / self.resolution), int((gy - self.origin_y) / self.resolution)

                if 0 <= ix < self.width and 0 <= iy < self.height:
                    if self.min_z <= gz <= self.max_z:
                        if 0 <= sx < self.width and 0 <= sy < self.height:
                            cv2.line(free_mask, (sx, sy), (ix, iy), 1, 1)
                        self.grid[iy, ix] = 100
                    elif gz < self.min_z:
                        if 0 <= sx < self.width and 0 <= sy < self.height:
                            cv2.line(free_mask, (sx, sy), (ix, iy), 1, 1)

            self.grid[(free_mask == 1) & (self.grid == -1)] = 0

        except Exception as e:
            pass

    def publish_pose(self):
        try:
            # 获取最新变换（Time(0) 在仿真模式下指向最新仿真时间）
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg() # 关键：使用节点仿真时钟
            pose.header.frame_id = self.odom_frame
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = t.transform.rotation
            self.pose_pub.publish(pose)
        except:
            pass

    def publish_map(self):
        if not self.is_mapping and not self.has_disk_map:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg() # 关键：使用节点仿真时钟
        msg.header.frame_id = self.odom_frame
        msg.info.resolution = self.resolution
        msg.info.width, msg.info.height = self.width, self.height
        
        if self.is_mapping and self.initial_pose_in_odom:
            tx_oi = self.initial_pose_in_odom.transform.translation.x
            ty_oi = self.initial_pose_in_odom.transform.translation.y
            q_oi = self.initial_pose_in_odom.transform.rotation
            yaw_oi = math.atan2(2 * (q_oi.w * q_oi.z + q_oi.x * q_oi.y), 1 - 2 * (q_oi.y * q_oi.y + q_oi.z * q_oi.z))
            msg.info.origin.position.x = tx_oi + self.origin_x * math.cos(yaw_oi) - self.origin_y * math.sin(yaw_oi)
            msg.info.origin.position.y = ty_oi + self.origin_x * math.sin(yaw_oi) + self.origin_y * math.cos(yaw_oi)
            msg.info.origin.orientation = q_oi
        else:
            msg.info.origin.position.x, msg.info.origin.position.y = self.origin_x, self.origin_y
            msg.info.origin.orientation.w = 1.0

        msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(msg)

    def start_mapping_callback(self, request, response):
        self.is_mapping = True
        self.has_disk_map = False
        try:
            self.initial_pose_in_odom = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time())
        except:
            self.initial_pose_in_odom = None
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        response.success = True
        response.message = "Mapping started with sync clock."
        return response

    def stop_mapping_callback(self, request, response):
        self.is_mapping = False
        response.success = self.save_map()
        return response

    def set_map_callback(self, request, response):
        m = request.map
        self.resolution, self.width, self.height = m.info.resolution, m.info.width, m.info.height
        self.origin_x, self.origin_y = m.info.origin.position.x, m.info.origin.position.y
        self.grid = np.array(m.data, dtype=np.int8).reshape((self.height, self.width))
        self.save_map()
        self.publish_map()
        response.success = True
        return response

    def load_map(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        pgm_path, yaml_path = os.path.join(dir_path, "map.pgm"), os.path.join(dir_path, "map.yaml")
        if not os.path.exists(pgm_path) or not os.path.exists(yaml_path): return
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                self.resolution = data['resolution']
                self.origin_x, self.origin_y = data['origin'][0], data['origin'][1]
            img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                self.height, self.width = img.shape
                img = np.flipud(img)
                self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
                self.grid[img < 10] = 100
                self.grid[img > 245] = 0
                self.has_disk_map = True
                self.get_logger().info("Map loaded from disk.")
        except Exception as e:
            self.get_logger().error(f"Load failed: {e}")

    def save_map(self):
        try:
            dir_path = os.path.dirname(os.path.realpath(__file__))
            pgm_path, yaml_path = os.path.join(dir_path, "map.pgm"), os.path.join(dir_path, "map.yaml")
            pgm_data = np.full((self.height, self.width), 205, dtype=np.uint8)
            pgm_data[self.grid == 100] = 0
            pgm_data[self.grid == 0] = 255
            cv2.imwrite(pgm_path, np.flipud(pgm_data))
            with open(yaml_path, 'w') as f:
                f.write(f"image: map.pgm\nresolution: {self.resolution}\norigin: [{self.origin_x}, {self.origin_y}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
            return True
        except: return False

def main(args=None):
    # 关键：传入命令行参数以接收 use_sim_time:=true
    rclpy.init(args=args)
    node = CloudToMapAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
