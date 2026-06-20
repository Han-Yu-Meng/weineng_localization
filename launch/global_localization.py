import os
import sys
import argparse
import subprocess
from fins import Node, Group, Agent, DefaultSource
from sensor import sensor_group
from gridmap import gridmap_group
from fastlio import fastlio_group

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
    parser = argparse.ArgumentParser(description="全局定位启动脚本")
    parser.add_argument("--map", type=str, default="YunJing_Airy",
                        help="地图名称")
    args = parser.parse_args()

    map_name = args.map
    home_dir = os.path.expanduser("~")
    map_base = os.path.join(home_dir, "Map", map_name)

    # geojson_file = os.path.join(map_base, "geojson", "graph.json")
    # geojson_publisher_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "geojson_publisher.py")
    # geojson_proc = subprocess.Popen(
    #     [sys.executable, geojson_publisher_script, geojson_file],
    #     start_new_session=True,
    # )

    with Agent(name="global_localization") as agent:
        agent.add_config("config/fastlio.yaml")
        agent.add_config("config/preprocess.yaml")
        agent.add_config("config/global_localization.yaml",
                         overrides = {
                             "GlobalLocalization.voxel_dir": os.path.join(map_base, "voxelmap"),
                             "GlobalLocalization.map_dir": os.path.join(map_base, "map", "map.pcd"),
                         })
        # agent.enable_debugging(full_backtrace = True)

        with DefaultSource("weineng_localization"):
            agent.launch(
                sensor_group(),
                gridmap_group(map_name=map_name),
                fastlio_group(),
                global_localization_group()
            )

        agent.spin()