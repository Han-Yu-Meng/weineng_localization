from fins import Node, Group, LaunchDescription, Agent, DefaultSource

def sensor_group():
    return Group([
        Node(
            package="ros_bridge",
            name="TransformRPY",
            outputs={
                "transform": "/tf_base_link_base_lidar",
            },
            parameters={
                "tx": "0.08521",
                "ty": "0.000000",
                "tz": "0.0333",
                "roll": "0.000000", # 角度值
                "pitch": "40.0",
                "yaw": "0.000000",
                "from_frame": "base_link",
                "to_frame": "base_lidar",
            },
        ),
        Node(
            package="ros_bridge",
            name="TFBroadcaster",
            inputs={
                "transform": "/tf_base_link_base_lidar",
            },
            parameters={
                "from_frame_override": "base_link",
                "to_frame_override": "base_lidar",
            },
        ),
        Node(
            package="ros_bridge",
            name="PointCloudSubscriber",
            outputs={
                "msg": "/rslidar_points_raw",
            },
            parameters={
                "topic": "/rslidar_points",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
        ),
        Node(
            package="ros_bridge",
            name="ImuSubscriber",
            outputs={
                "msg": "/rslidar_imu_data",
            },
            parameters={
                "topic": "/rslidar_imu_data",
                "history": "Keep Last",
                "depth": "100",
                "reliability": "Best Effort",
                "durability": "Volatile",
            },
        ),
        Node(
            package="airy_preprocess",
            name="AiryPreprocess",
            inputs={
                "lidar": "/rslidar_points_raw",
                "$T_{target}^{lidar}$": "/tf_base_link_base_lidar",
            },
            outputs={
                "output_cloud": "/rslidar_points",
                "filter_box_marker": "/filter_box_marker",
            },
        ),
        Node(
            package="ros_bridge",
            name="MarkerArrayPublisher",
            inputs={
                "msg": "/filter_box_marker",
            },
            parameters={
                "topic": "/filter_box_marker",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
        ),
    ])