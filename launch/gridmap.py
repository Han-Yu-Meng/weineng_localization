from fins import Node, Group

def generate_gridmap_group():
    return Group([
        Node(
            package="gridmap_server",
            name="GridMapServer",
            outputs={
                "global_grid_map": "/global_grid_map",
            },
            parameters={
                "map_yaml_path": "/home/fins/Map/YunJing_Airy/map/map.yaml",
            },
        ),
        Node(
	    package="ros_bridge",
	    name="OccupancyGridPublisher",
	    inputs={
		"msg": "/global_grid_map",
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
    
