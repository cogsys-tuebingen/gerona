# Path Planner

Receives goal descriptions by [path_control](path_control) or directly by some high level node and finds a path on the map.
The map is optional and has to be provided by some external node.
The currently seen obstacles are integrated into the map before the search begins.
If no map is available, only the obstacles are used for planning.

# Kinematic Path Planner

The node ``path_planner_node`` performs A* search with a car-like kinematic model.
Other kinematic models are also supported, however they are statically compiled.

## Parameters

### Common Parameters
| Name | Type | Default | Description |
| -------- | -------- | -------- | -------- |
| ~target_topic | string | /goal | Topic to subscribe for goal specification. |
| ~use_map_topic | bool | false | If false, use service to request map, otherwise subscribe map topic. |
| ~map_topic | string | /map | Name of the topic, that provides the map (only used if use_map_topic=true). |
| ~map_service | string | /dynamic_map | Name of the service, that provides the map (only used if use_map_topic=false). |
| ~base_frame | string | /base_link | Name of the robot frame. |
| ~algorithm | string | generic | The type of kinematic model to use. Can be one of ``generic, ackermann, 2d`` or also more specialized variants: ``summit, patsy, patsy_forward, summit_forward``. |
| ~grow_obstacles | double | 0.0 | Radius which obstacles are grown by. |
| ~use_cloud | bool | true | When true, obstacle points in ``/obstacle_cloud`` are integrated into the map before path planning. |
| ~use_scan_front  | bool | false | When true, laser measurements in ``/scan/front/filtered`` are integrated into the map before path planning. |
| ~use_scan_back | bool | false |  When true, laser measurements in ``/scan/back/filtered`` are integrated into the map before path planning. |
| ~use_cost_map | bool | false | When true, use the costmap published on ``cost_map_service`` for optimization. |
| ~use_unknown_cells | bool | true | Allow planning through unknown cells. |
| ~oversearch_distance | double | 0.0 | Specify for how long the search continues after a first candidate solution has been found. The search stops at the first node that has a distance to the start larger than the first candidate + ``oversearch_distance``. |
| ~penalty/backward | double | 2.5 | Penalty weight for path segments that have to be traversed in reverse. |
| ~penalty/turn | double | 4.0 | Penalty weight for switching direction on the path. |
| ~preprocess | bool | true | If true, the costmap for optimization is generated from the map using distance transform. |
| ~postprocess | bool | true | If true, the resulting path is interpolated and smoothed. |
| ~render_open_cells | bool | false | If true, the list of open cells is periodically published as grid cells. |

### Collision Model
| Name | Type | Default | Description |
| -------- | -------- | -------- | -------- |
| ~use_collision_gridmap | bool | false | When true, the search uses a box model for collision checking (parameter ``size``). Otherwise the robot is treated as a point. |
| ~size/backward | double | -0.6 | Distance from the center to the rear most point in the (forward direction.) |
| ~size/forward | double | 0.4 | Distance from the center to the front most point. |
| ~size/width | double | 0.5 | Width of the robot. |

### Optimization Parameters (unstable)
| Name | Type | Default | Description |
| -------- | -------- | -------- | -------- |
| ~optimization/optimize_cost | bool | false | Activate path optimization (unstable.) Minimizes the a weighted sum of a cost map, the original path and a smoothness term |
| ~optimization/publish_gradient | bool | false | Publish a vector field as Markers representing the optimization gradient. |
| ~optimization/tolerance | double | 1e-5 | Maximum change in one iteration of the optimization considered as converged. |
| ~optimization/weight/cost | double | 0.75 | Weight of the cost term in optimization. |
| ~optimization/weight/data | double | 0.9 | Weight of the original path. |
| ~optimization/weight/smooth | double | 0.3 | Smoothness weight. |


## Topics
### Subscriptions
| Name | Type | Description |
| -------- | -------- | ---------------- |
| goal (param: ~target_topic) | geometry_msgs/PoseStamped | Goal pose |
| map (param: ~map_topic) | nav_msgs/OccupancyGrid | Map |
| obstacle_cloud | sensor_msgs/PointCloud2 | (optional) Obstacles |
| scan/front/filtered | sensor_msgs/LaserScan | (optional) Front facing laser scanner |
| scan/back/filtered | sensor_msgs/LaserScan | (optional) Rear facing laser scanner |
| tf | | |

### Publications
| Name | Type | Description |
| -------- | -------- | ---------------- |
| path | [path_msgs/PathSequence](path-msgs) | The final (refined) path |
| path_raw | nav_msgs/Path | Raw, unrefined path |
| visualization_marker | visualization_msgs/Marker | Individual Markers |
| visualization_marker_array | visualization_msgs/MarkerArray | Marker Arrays |
| rosout | rosgraph_msgs/Log | ROS logging |
| ~cells | nav_msgs/GridCells | List of open cells, if ``~render_open_cells`` is true |
| ~cost | nav_msgs/OccupancyGrid | The currently used cost map. |
| ~map | nav_msgs/OccupancyGrid | The final map used for planning after integrating all obstacle information. |
