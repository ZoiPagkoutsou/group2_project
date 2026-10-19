# create3_ws — running the robot (robot_8)

All commands to run the iRobot Create 3 (`/robot_8`) with ROS 2 Jazzy.
Every terminal needs its own `source` lines.

## 0. Check the setup (once per terminal)

```bash
echo $ROS_DOMAIN_ID          # must be 8
echo $RMW_IMPLEMENTATION     # must be rmw_cyclonedds_cpp
ros2 topic list | grep robot_8   # robot topics must show up
```

## 1. Build (after every code change)

```bash
cd ~/create3_ws
colcon build --packages-select create3_pkg
source install/setup.bash
```

Always build from `~/create3_ws`, never from inside `src/`.

## 2. Terminal 1 — start the LiDAR (always first, leave it running)

```bash
source /opt/ros/jazzy/setup.bash
source ~/create3_examples_ws/install/setup.bash
ros2 launch create3_lidar_slam sensors_launch.py namespace:="robot_8"
```

Check that scans arrive (about 5–10 Hz), then Ctrl+C:

```bash
ros2 topic hz /robot_8/scan
```

## 3. Terminal 2 — run the robot

```bash
source /opt/ros/jazzy/setup.bash
cd ~/create3_ws
source install/setup.bash
```

Then pick **one** of the modes below.

### Mode A — go from A to B (go_to_goal)

Place the robot at A with the bumper pointing to "forward", then:

```bash
ros2 launch create3_pkg go_to_goal.launch.py goal_x:=2.0 goal_y:=1.0
```

| Argument | Meaning | Default |
|---|---|---|
| `goal_x` | metres forward of the start (negative = behind) | required |
| `goal_y` | metres left of the start (negative = right) | required |
| `goal_tolerance` | stop when this close to B (m) | `0.10` |
| `auto_side` | `true`: choose the side for each obstacle from the LiDAR; `false`: always `follow_side` | `true` |
| `follow_side` | side used to go round obstacles when `auto_side:=false` (fallback otherwise): `right` / `left` | `right` |

Examples:

```bash
ros2 launch create3_pkg go_to_goal.launch.py goal_x:=0.5 goal_y:=0.0 goal_tolerance:=0.03   # 0.5 m ahead, precise
ros2 launch create3_pkg go_to_goal.launch.py goal_x:=1.0 goal_y:=-1.0                       # 1 m ahead, 1 m right
ros2 launch create3_pkg go_to_goal.launch.py goal_x:=-1.0 goal_y:=0.0                       # 1 m behind
```

The robot drives straight at B. When an obstacle is in the way it looks
with the LiDAR for the obstacle's ends and goes round the shorter side, and
it leaves the wall as soon as there is a free way towards B (DistBug). It
stops at B (`GOAL REACHED`); if B cannot be reached it stops with
`GOAL UNREACHABLE`.

### Mode B — wall following only

```bash
ros2 launch create3_pkg maze.launch.py standalone:=true
ros2 launch create3_pkg maze.launch.py standalone:=true follow_side:=left
```

### Mode C — full Pledge navigation system

Starts wall_follower (perception only), heading_publisher,
navigation_maze_solving and movement_controller_new:

```bash
ros2 launch create3_pkg maze.launch.py
ros2 launch create3_pkg maze.launch.py ir_threshold:=600
```

> Status: needs the open fixes in `heading_publisher.py` before the robot
> will move in this mode.

## Nodes started by each launch file

| Launch command | Node (ROS name) | Executable / file | Package / owner |
|---|---|---|---|
| `sensors_launch.py namespace:="robot_8"` (terminal 1) | `/robot_8/rplidar_node` | `rplidar_composition` | `rplidar_ros` (LiDAR driver) |
| | `/robot_8/static_transform_publisher` | `static_transform_publisher` | `tf2_ros` (LiDAR position) |
| `go_to_goal.launch.py` (Mode A) | `/robot_8/wall_follower` | `wall_follower_node` — `wall_follower_node.py` + `goal_controller.py`, `controller.py`, `lidar_processing.py` | `create3_pkg` |
| `maze.launch.py standalone:=true` (Mode B) | `/robot_8/wall_follower` | `wall_follower_node` — `wall_follower_node.py` + `controller.py`, `lidar_processing.py` | `create3_pkg` |
| `maze.launch.py` (Mode C) | `/robot_8/wall_follower` (perception only, `drive:=false`) | `wall_follower_node` — `wall_follower_node.py` | `create3_pkg` |
| | `/heading_publisher` | `heading_publisher` — `heading_publisher.py` | `create3_pkg` |
| | `/navigation_node` | `navigation_maze_solving` — `navigation_maze_solving.py` | `create3_pkg` |
| | `/movement_controller` | `movement_controller_new` — `movement_controller_new.py` | `create3_pkg` |

### What each node reads and sends

| Node | Subscribes | Publishes / uses |
|---|---|---|
| `wall_follower` (Mode A) | `/robot_8/scan`, `/robot_8/odom` | `/robot_8/cmd_vel`, `~/state`, `~/distances`, `~/situation` |
| `wall_follower` (Mode B) | `/robot_8/scan` | `/robot_8/cmd_vel`, `~/state`, `~/distances`, `~/situation` |
| `wall_follower` (Mode C) | `/robot_8/scan` | `~/distances`, `~/situation`, `~/state` (no driving) |
| `heading_publisher` | `/robot_8/odom` | `/robot_8/heading`, `/robot_8/accumulated_heading` |
| `navigation_node` | `/robot_8/wall_follower/distances`, `/robot_8/heading`, `/movement_status` | `/movement_command` |
| `movement_controller` | `/movement_command`, `/robot_8/ir_intensity`, `/robot_8/hazard_detection` | `/movement_status`, actions `/robot_8/drive_distance`, `/robot_8/rotate_angle` |

`~/` = `/robot_8/wall_follower/`. Check the running nodes with `ros2 node list`
and one node's connections with `ros2 node info /robot_8/wall_follower`.

## 4. Terminal 3 — watch what the robot is doing (optional)

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /robot_8/wall_follower/state       # GO_TO_GOAL, FOLLOW_WALL(left)/..., GOAL_REACHED ...
ros2 topic echo /robot_8/wall_follower/distances   # [front, left, right, front_left, front_right] m
ros2 topic hz /robot_8/odom                        # odometry must arrive
ros2 topic echo /movement_command                  # Mode C only
ros2 topic echo /movement_status                   # Mode C only
```

## 5. Stop

Press **Ctrl+C** in terminal 2 — the robot stops. It also stops by itself
when LiDAR or odometry data stop arriving for 0.5 s.

## Troubleshooting

| Message / problem | Fix |
|---|---|
| `Waiting for LiDAR data...` | Start the LiDAR (step 2) and check `ros2 topic hz /robot_8/scan` |
| Robot does not move in Mode A, no `Start pose fixed` line | No odometry: check `ros2 topic hz /robot_8/odom` |
| `Package 'create3_pkg' not found` | `source install/setup.bash` in `~/create3_ws` (after building) |
| No robot topics in `ros2 topic list` | Check `ROS_DOMAIN_ID=8`, same WiFi as the robot, robot fully booted |
| Stops a bit short of B | Lower the tolerance: `goal_tolerance:=0.03` |
| Code change has no effect | Rebuild (step 1) — the installed copy is what runs |
