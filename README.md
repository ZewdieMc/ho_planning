# ho_planning - ROS Package

This ROS package is designed for robot planning tasks, including state machine execution, frontier-based exploration, global path planning, and local path following using DWA algorithm.

## How to Run

# Package is built uisng catkin tools.
catkin_make has some issues. Fix otherwise.
```sh
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

1. **Launch the Stonefish Simulation**
   - Start the simulation environment and required modules:
     ```sh
     roslaunch ho_planning stonefish.launch
     ```

2. **Launch the Planning Modules**
   - Activate all four planning modules:
     ```sh
     roslaunch ho_planning planning.launch
     ```

3. **Visualize in RViz**
   - Open RViz with the provided configuration file:
     ```sh
     sudo apt-get install ros-<your-ros-distro>-smach-viewer
     rviz -d $(rospack find ho_planning)/rviz/planning.rviz
     ```

4. **Visualize the State Machine**
   - Use `smach_viewer` to visualize the state machine:
     ```sh
     rosrun smach_viewer smach_viewer.py
     ```

5. **Start the Exploration Process**
   - Trigger the exploration process by calling the `/start_exploration` service in the state machine node:
     ```sh
     rosservice call /start_exploration "data: true"
     ```

## Nodes

### 1. State Machine Node (`state_machine.py`)

#### Purpose
Execution node that utilizes all other modules to execute the exploration task.

#### Subscribers

- **Grid Map**
  - **Topic**: `/projected_map`
  - **Message Type**: `OccupancyGrid`
  - **Callback Function**: `self.get_gridmap`
  - **Description**: Subscribes to grid map from Octomap Server.

- **Odometry**
  - **Topic**: `/odom`
  - **Message Type**: `Odometry`
  - **Callback Function**: `self.get_odom`
  - **Description**: Subscribes to odometry data.

- **Move Base Goal**
  - **Topic**: `/move_base_simple/goal`
  - **Message Type**: `PoseStamped`
  - **Callback Function**: `self.get_goal`
  - **Description**: Subscribes to goals published by RViz.

- **Aruco Pose**
  - **Topic**: `/aruco_pose`
  - **Message Type**: `PoseStamped`
  - **Callback Function**: `self.aruco_cb`
  - **Description**: Subscribes to Aruco marker pose.

#### Services

- **Stop State Machine**
  - **Service Name**: `/stop_state_machine`
  - **Service Type**: `SetBool`
  - **Handler Function**: `self.handle_stop_srv`
  - **Description**: Service to preempt the state machine.

- **Start Exploration**
  - **Service Name**: `/start_exploration`
  - **Service Type**: `SetBool`
  - **Handler Function**: `self.handle_start_exploration`
  - **Description**: Service to start the exploration process.

- **Start PickPlace**
  - **Service Name**: `/start_pickplace`
  - **Service Type**: `SetBool`
  - **Handler Function**: `self.handle_start_pickplace`
  - **Description**: Service to start the pick and place task.

#### Action Client

- **Follow Path**
  - **Action Name**: `follow_path`
  - **Action Type**: `FollowPathAction`
  - **Client Variable**: `self.client`
  - **Description**: Client for following the planned path using DWA.

### 2. Frontier-Based Exploration Node (`frontier_planner_node.py`)

#### Purpose
Frontier-based exploration algorithm to find the next best viewpoint.

#### Publishers

- **Frontiers**
  - **Topic**: `~frontiers`
  - **Message Type**: `GridCells`
  - **Queue Size**: 1
  - **Description**: Publishes detected frontiers.

- **Viewpoints**
  - **Topic**: `~viewpoints`
  - **Message Type**: `Marker`
  - **Queue Size**: 1
  - **Description**: Publishes potential viewpoints.

- **NBVP**
  - **Topic**: `~nbvp`
  - **Message Type**: `Marker`
  - **Queue Size**: 1
  - **Description**: Publishes next best viewpoint.

- **Goal**
  - **Topic**: `/move_base_simple/goal`
  - **Message Type**: `PoseStamped`
  - **Queue Size**: 1
  - **Description**: Publishes goal for the move base.

#### Subscribers

- **Odometry**
  - **Topic**: `odom_topic`
  - **Message Type**: `Odometry`
  - **Callback Function**: `self.odom_cb`
  - **Description**: Subscribes to odometry data.

- **Grid Map**
  - **Topic**: `gridmap_topic`
  - **Message Type**: `OccupancyGrid`
  - **Callback Function**: `self.gridmap_cb`
  - **Description**: Subscribes to grid map data.

#### Services

- **Get NBVP**
  - **Service Name**: `/get_nbvp`
  - **Service Type**: `GetNBVP`
  - **Handler Function**: `self.handle_get_nbvp`
  - **Description**: Service to get the next best viewpoint.

- **Get Viewpoints**
  - **Service Name**: `/get_viewpoints`
  - **Service Type**: `GetViewPoints`
  - **Handler Function**: `self.handle_get_viewpoints`
  - **Description**: Service to get viewpoints.

### 3. Global Planner Node (`global_planner_node.py`)

#### Purpose
RRT* node to plan a global path to a given goal.

#### Publishers

- **RRT Nodes**
  - **Topic**: `~rrt_nodes`
  - **Message Type**: `Marker`
  - **Queue Size**: 1
  - **Description**: Publishes RRT nodes.

- **RRT Edges**
  - **Topic**: `~rrt_edges`
  - **Message Type**: `Marker`
  - **Queue Size**: 1
  - **Description**: Publishes RRT edges.

- **RRT Bounds**
  - **Topic**: `~rrt_bounds`
  - **Message Type**: `Marker`
  - **Queue Size**: 1
  - **Description**: Publishes RRT boundaries.

#### Subscribers

- **Grid Map**
  - **Topic**: `gridmap_topic`
  - **Message Type**: `OccupancyGrid`
  - **Callback Function**: `self.get_gridmap`
  - **Description**: Subscribes to grid map data.

- **Odometry**
  - **Topic**: `odom_topic`
  - **Message Type**: `Odometry`
  - **Callback Function**: `self.get_odom`
  - **Description**: Subscribes to odometry data.

#### Services

- **Get Global Plan**
  - **Service Name**: `get_global_plan`
  - **Service Type**: `GetGlobalPlan`
  - **Handler Function**: `self.handle_get_global_plan`
  - **Description**: Service to get the global plan.

### 4. DWA Node (`dwa_node.py`)

#### Purpose
Local planner that uses the DWA algorithm to follow the given global path.

#### Action Server

- **Follow Path**
  - **Action Name**: `follow_path`
  - **Action Type**: `FollowPathAction`
  - **Server Variable**: `self.server`
  - **Description**: Action server for following the planned path using DWA.

#### Publishers

- **Velocity**
  - **Topic**: `/cmd_vel`
  - **Message Type**: `Twist`
  - **Queue Size**: 1
  - **Description**: Publishes velocity commands.

- **States**
  - **Topic**: `~states`
  - **Message Type**: `Marker`
  - **Queue Size**: 1
  - **Description**: Publishes state markers.

- **Trajectory**
  - **Topic**: `~local_path`
  - **Message Type**: `Marker`
  - **Queue Size**: 1
  - **Description**: Publishes the local path trajectory.

- **Closest Obstacle**
  - **Topic**: `~closest_obs`
  - **Message Type**: `Marker`
  - **Queue Size**: 1
  - **Description**: Publishes markers for the closest obstacles.

- **Filtered Scan**
  - **Topic**: `/scan_filtered`
  - **Message Type**: `LaserScan`
  - **Queue Size**: 1
  - **Description**: Publishes filtered laser scan data.

#### Subscribers

- **Odometry**
  - **Topic**: `odom_topic`
  - **Message Type**: `Odometry`
  - **Callback Function**: `self.odom_cb`
  - **Description**: Subscribes to odometry data.

- **Grid Map**
  - **Topic**: `gridmap_topic`
  - **Message Type**: `OccupancyGrid`
  - **Callback Function**: `self.gridmap_cb`
  - **Description**: Subscribes to grid map data.

- **Laser Scan**
  - **Topic**: `/turtlebot/k


# Note: 
