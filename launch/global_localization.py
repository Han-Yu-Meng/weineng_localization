from fins import Node, Group, LaunchDescription, Agent, DefaultSource
from fastlio import generate_fastlio_group
from gridmap import generate_gridmap_group
import os
import subprocess

def generate_global_localization_group():
    return Group([
        Node(
            package="global_localization",
            name="GlobalLocalization",
            inputs={
                "cloud": "/cloud_registered",
                "$T_{odom}^{baselink}$": "/tf_odom_base",
            },
            outputs={
                "global_map_viz": "/localization/global_map_viz_internal",
                "aligned_cloud": "/localization/aligned_cloud_internal",
                "current_pose": "/localization/localization_pose",
                "$T_{map}^{odom}$": "/tf/map_to_odom"
            },
        ),
        Node(
            package="ros_bridge",
            name="PoseStampedPublisher",
            inputs={
                "msg": "/localization/localization_pose",
            },
            parameters={
                "topic": "/localization_pose",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Volatile",
            },
        ),
        Node(
            package="ros_bridge",
            name="TFBroadcaster",
            parameters={"from_frame_override": "map", "to_frame_override": "odom"},
            inputs={"transform": "/tf/map_to_odom"}
        ),
        Node(
            package="pointcloud_converter",
            name="PCL2ROS",
            inputs={
                "pcl_cloud": "/localization/aligned_cloud_internal", 
            },
            outputs={
                "ros_cloud": "/viz/ros_aligned_cloud"
            }
        ),
        Node(
            package="ros_bridge",
            name="PointCloudPublisher",
            inputs={
                "msg": "/viz/ros_aligned_cloud",
            },
            parameters={
                "topic": "/aligned_cloud",
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
                "pcl_cloud": "/localization/global_map_viz_internal", 
            },
            outputs={
                "ros_cloud": "/viz/ros_global_map"
            }
        ),
        Node(
            package="ros_bridge",
            name="PointCloudPublisher",
            inputs={
                "msg": "/viz/ros_global_map",
            },
            parameters={
                "topic": "/global_map_viz",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Transient Local",
            },
        ),
    ])

def generate_launch():
    return LaunchDescription(groups=[
        generate_global_localization_group(),
        generate_fastlio_group(),
        generate_gridmap_group()
    ])

if __name__ == "__main__":
    with Agent(name="global_localization", port=7777) as agent:
        with DefaultSource("weineng_localization"):
            ld = generate_launch()
        
        agent.add_config("config/global_localization.yaml")
        agent.add_config("config/fastlio.yaml")

        agent.launch(ld)
        """
        bag_name = "rosbag2_2026_06_01-20_58_32"
        if os.path.exists(bag_name):
            print(f"Playing {bag_name}...")
            subprocess.Popen(["ros2", "bag", "play", bag_name])
        else:
            print(f"{bag_name} not found, skip playing.")"""
            
        agent.spin()
