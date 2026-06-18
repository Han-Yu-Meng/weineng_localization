from fins import Node, Group

def gridmap_group():
    return Group([
        Node(
            package="gridmap_server",
            name="GridMapServer",
            outputs={
                "global_grid_map": "/map",
            },
            parameters={
                "map_yaml_path": "/home/fins/Map/Weineng/map/map.yaml",
            },
        ),
        Node(
            package="ros_bridge",
            name="OccupancyGridPublisher",
            inputs={
                "msg": "/map",
            },
            parameters={
                "topic": "/map",
                "history": "Keep Last",
                "depth": "10",
                "reliability": "Reliable",
                "durability": "Transient Local",
            },
        ),
    ])
    
