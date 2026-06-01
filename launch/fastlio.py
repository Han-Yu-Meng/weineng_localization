from fins import Node, Group, LaunchDescription, Agent, DefaultSource

def generate_fastlio_group():
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
            name="PointCloudSubscriber",
            outputs={
                "msg": "/rslidar_points",
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
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
        ),
        Node(
            package="FAST_LIO",
            name="FastLIO",
            inputs={
                "imu": "/rslidar_imu_data",
                "lidar": "",
                "lidar_standard": "/rslidar_points",
                "$T_{base}^{lidar}$": "/tf_base_link_base_lidar",
            },
            outputs={
                "cloud": "/cloud_registered",
                "cloud_body": "/cloud_registered_body",
                "path": "/path",
                "odometry": "/Odometry",
                "$T_{odom}^{base}$": "/tf_odom_base",
            },
        ),
        Node(
            package="pointcloud_converter",
            name="PCL2ROS",
            inputs={
                "pcl_cloud": "/cloud_registered",
            },
            outputs={
                "ros_cloud": "/cloud_registered_ros",
            },
        ),
        Node(
            package="ros_bridge",
            name="PointCloudPublisher",
            inputs={
                "msg": "/cloud_registered_ros",
            },
            parameters={
                "topic": "/cloud_registered",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
        ),
        Node(
            package="pointcloud_converter",
            name="PCL2ROS",
            inputs={
                "pcl_cloud": "/cloud_registered_body",
            },
            outputs={
                "ros_cloud": "/cloud_registered_body_ros",
            },
        ),
        Node(
            package="ros_bridge",
            name="PointCloudPublisher",
            inputs={
                "msg": "/cloud_registered_body_ros",
            },
            parameters={
                "topic": "/cloud_registered_body",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
        ),
        Node(
            package="ros_bridge",
            name="TFBroadcaster",
            inputs={
                "transform": "/tf_odom_base",
            },
        ),
        Node(
	    package="ros_bridge",
	    name="OdometryPublisher",
	    inputs={
		"msg": "/Odometry",
	    },
	    parameters={
		"topic": "/Odometry",
		"history": "Keep Last",
		"depth": "10",
		"reliability": "Reliable",
		"durability": "Volatile",
	    },
	),
	#Node(
	#    package="ros_bridge",
	#    name="TFLogger",
	#    inputs={
	#	"transform": "/tf_odom_base",
	#    },
	#),
    ])
    
def generate_map_odom_group():
    return Group([
        Node(
            package="ros_bridge",
            name="TransformRPY",
            outputs={
                "transform": "/tf_map_odom",
            },
            parameters={
                "tx": "0.000000",
                "ty": "0.000000",
                "tz": "0.000000",
                "roll": "0.000000",
                "pitch": "0.000000",
                "yaw": "0.000000",
                "from_frame": "map",
                "to_frame": "odom",
            },
        ),
        Node(
            package="ros_bridge",
            name="TFBroadcaster",
            inputs={
                "transform": "/tf_map_odom",
            },
            parameters={
                "from_frame_override": "map",
                "to_frame_override": "odom",
            },
        ),
    ])
    

def generate_launch():
    return LaunchDescription(groups=[
        generate_fastlio_group(),
        generate_map_odom_group()
    ])


if __name__ == "__main__":
    with Agent(name="fastlio", port=1896) as agent:
        with DefaultSource("weineng_localization"):
            ld = generate_launch()
        
        agent.add_config("config/fastlio.yaml")

        agent.launch(ld)
        agent.spin()
