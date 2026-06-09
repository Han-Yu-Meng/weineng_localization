from fins import Node, Group, LaunchDescription, Agent, DefaultSource
from fastlio import fastlio_group
from gridmap import gridmap_group
from sensor import sensor_group
import os
import subprocess
import argparse

def global_localization_group():
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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Launch localization with optional bag playback.")
    parser.add_argument("--bag", type=str, help="Path to the ROS2 bag to play", default=None)
    args = parser.parse_args()

    with Agent(name="global_localization", port=2222) as agent:
        agent.add_config_dir("config")
        agent.log_level("INFO")
        # agent.enable_performance_monitor()
        
        with DefaultSource("weineng_localization"):
            agent.launch(
                sensor_group(),
                fastlio_group(),
                global_localization_group()
            )
        
        bag_process = None
        if args.bag:
            if os.path.exists(args.bag):
                print(f"Playing {args.bag}...")
                bag_process = subprocess.Popen(["ros2", "bag", "play", args.bag])
            else:
                print(f"Error: Bag file/directory '{args.bag}' not found.")
        else:
            print("No bag provided, skipping playback.")
            
        try:
            agent.spin()
        finally:
            if bag_process:
                print("Stopping bag playback...")
                bag_process.terminate()
                bag_process.wait()
