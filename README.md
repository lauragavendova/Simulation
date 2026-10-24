# robocup_ws

Isaac Sim simulation of the MRVK robot "shoddy" (4-wheel skid steer, 2D lidar)
in the RoboCup@Home HCC arena, with an optional Nav2 navigation stack.
ROS 2 Jazzy · Isaac Sim 6.2 · Ubuntu 24.04

This branch (`ros2-jazzy-isaac-sim`) ports the original ROS 2 Humble setup to
ROS 2 Jazzy, see [Migration from Humble to Jazzy](#migration-from-humble-to-jazzy).

## Requirements

- Ubuntu 24.04 with **ROS 2 Jazzy** (`/opt/ros/jazzy`, `ros-jazzy-desktop`)
- **Isaac Sim 6.2** standalone, by default in `~/isaacsim`
  (otherwise `export ISAACSIM_PATH=/path/to/isaacsim`)
- An NVIDIA GPU that runs Isaac Sim (Humble tested on an RTX 3080 Ti Laptop,
  16 GB; Jazzy on an RTX 3060 Laptop, 6 GB)

## Setup

```bash
git clone <this repo> ~/robocup_ws
cd ~/robocup_ws
sudo apt install python3-rosdep ros-jazzy-libg2o
sudo rosdep init   # only once per machine
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
```

The ROS distribution defaults to Jazzy; another one can be picked with
`ROS_DISTRO=humble ./run.sh` (the C++ changes below build on both, the Nav2
and slam_toolbox config targets Jazzy).

The workspace is built automatically on the first `./run.sh`
(or explicitly with `./run.sh --build`).

## Running

```bash
./run.sh                        # arena + robot, RViz, steering GUI
./run.sh --nav                  # + Nav2 and slam_toolbox (TEB local planner)
./run.sh --nav --planner dwb    # local planner: teb (default) | dwb | tr
./run.sh --headless             # Isaac without a window (RViz still opens)
./run.sh --x 2 --y -1 --yaw 1.57    # spawn pose
./run.sh --world other.usd      # another USD environment
./run.sh --build                # rebuild the workspace first
```

`run.sh` starts Isaac Sim, waits until it publishes `/clock` (the first start
compiles shaders and can take a few minutes), then starts the ROS 2 side.
Ctrl+C, or closing either side, stops both.

With `--nav`, give the robot a goal with **2D Goal Pose** in RViz. Driving by
hand works through the rqt_robot_steering window (it can stay open next to
Nav2; it sends one zero when released).

### Environment

Both sides run in a clean environment containing only `/opt/ros/$ROS_DISTRO`
(default `jazzy`) and this workspace, so a shell sourced for another workspace cannot leak in:

- Isaac Sim's Python 3.12 must not see the system ROS paths: its ROS 2 bridge
  then falls back to its own internal libraries for the distro matching the
  Ubuntu version (Jazzy on 24.04). For that reason `ROS_DISTRO` itself is not
  passed to Isaac; the Isaac script gets `ROBOCUP_ROS_DISTRO` instead (used
  only to run `xacro`).
- Isaac's ROS 2 bridge uses **Fast DDS**; the ROS side is pinned to
  `rmw_fastrtps_cpp`. To inspect topics from your own terminal:
  `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

## ROS interface

| Topic / frame | Direction | Notes |
|---|---|---|
| `/clock` | Isaac → | simulation time, everything runs with `use_sim_time` |
| `/odom`, TF `odom → base_link` | Isaac → | exact pose from the physics (no drift) |
| `/scan_raw` | Isaac → | RTX lidar, 360°, 0.1°, 10 Hz, range 0.15–4 m, frame `lidar_frame` |
| `/scan` | relay → | `/scan_raw` with no-hit beams turned from 0.0 into `inf` |
| `/cmd_vel` | → Isaac | `geometry_msgs/Twist` |
| TF `base_link → …` | robot_state_publisher | from `robocup_description` |
| TF `map → odom`, `/map` | slam_toolbox (`--nav`) | visualisation only |

Nav2 navigates in the `odom` frame with rolling-window costmaps fed by `/scan`;
there is no static map or global localisation.

## Packages

- `src/robocup_sim` – Isaac Sim script (`isaac/robocup_isaac.py`), launch
  files, Nav2 / slam_toolbox / RViz configs, the arena (`worlds/`).
- `src/robocup_description` – URDF/xacro and meshes of shoddy.
- `src/teb_local_planner`, `src/costmap_converter` – vendored from
  [rst-tu-dortmund](https://github.com/rst-tu-dortmund) (`humble-devel` /
  `ros2`); TEB has no Humble/Jazzy binaries. Both are patched to build on
  Humble and Jazzy, see below.

## Simulation notes

- **Kinematic base.** `/cmd_vel` is applied directly as the chassis velocity
  (acceleration-limited to 2 m/s²) and the chassis is kept level; the wheels are
  frictionless. The robot still collides with the world. A PhysX 4-wheel skid
  steer with this URDF chattered, flipped and could not turn on the spot.
- **Arena.** The USD (Blender export, Y-up, centimetres) is turned Z-up, scaled
  to metres and centred on the origin. Every mesh gets a triangle-mesh
  collider; the two sliding doors get box colliders instead. The ceiling is
  hidden. Spawn (0, 0) is free floor.
- **Lidar** stands in for the robot's Hokuyo URG-04LX (which scanned 240° with
  1 cm noise; the simulated one scans 360° without noise or motion distortion).
  It sees rendered geometry, not colliders.
- Footprint for Nav2 is the rectangle over the bumpers and wheels,
  0.56 × 0.47 m.

## Migration from Humble to Jazzy

Changes made on this branch compared to the original Humble version. The
script and C++ changes are written to work on Humble too; the Nav2 config and
the slam_toolbox launch changes target Jazzy and were not tested on Humble.

### Build and startup scripts

| File | Change | Why |
|---|---|---|
| `run.sh` | `/opt/ros/humble` replaced with `/opt/ros/$ROS_DISTRO`, `ROS_DISTRO` defaults to `jazzy`; Isaac gets `ROBOCUP_ROS_DISTRO` | Distro was hard-coded |
| `src/robocup_sim/isaac/robocup_isaac.py` | `xacro` is run with `/opt/ros/$ROBOCUP_ROS_DISTRO/setup.bash` (default `jazzy`) | Same |

The Isaac Sim side needs no other change: on Ubuntu 24.04 the
`isaacsim.ros2.bridge` loads its internal Jazzy libraries by itself
(`Attempting to load internal rclpy for ROS Distro: jazzy` in the log).

### Vendored C++ packages

| File | Change | Why |
|---|---|---|
| `costmap_converter/.../background_subtractor.h`, `costmap_to_dynamic_obstacles.h` | `#if __has_include(<cv_bridge/cv_bridge.hpp>)`, otherwise `cv_bridge.h` | Jazzy's cv_bridge ships only `cv_bridge.hpp`, Humble's only `cv_bridge.h` |
| `teb_local_planner/src/teb_local_planner_ros.cpp` | Includes `nav2_core/controller_exceptions.hpp` and throws `nav2_core::ControllerException` when available, otherwise the old `exceptions.hpp` / `PlannerException` | `nav2_core/exceptions.hpp` no longer exists in Jazzy |
| `teb_local_planner/src/teb_local_planner_ros.cpp` | Helper `calculateMinAndMaxDistances(...)` that calls whichever Nav2 overload exists | In Jazzy `nav2_costmap_2d::calculateMinAndMaxDistances` returns `std::pair` instead of using output parameters |
| `teb_local_planner/include/teb_local_planner/misc.h` | Own `sign()` and `average_angle()` in namespace `teb_local_planner`; `g2o::sign` / `g2o::average_angle` calls replaced (`pose_se2.h`, `edge_kinematics.h`, `edge_obstacle.h`, `optimal_planner.cpp`, `recovery_behaviors.cpp`, `teb_local_planner_ros.cpp`) | `ros-jazzy-libg2o` is a newer g2o whose `g2o/stuff/misc.h` no longer has these helpers |

### Nav2 configuration (`src/robocup_sim/config/nav2/nav2_params_*.yaml`)

- Removed `bt_navigator.plugin_lib_names`. Since Jazzy the default BT node
  libraries are loaded automatically, listing them again makes bt_navigator
  fail with a "node already registered" error.
- Plugin names use `::` instead of `/`: `nav2_navfn_planner::NavfnPlanner`,
  `nav2_behaviors::Spin|BackUp|DriveOnHeading|Wait`. Jazzy only knows the
  class names; with the old names `planner_server` fails with *"class
  nav2_navfn_planner/NavfnPlanner ... does not exist"* and the whole Nav2
  bringup aborts.
- DWB (`dwb` and `tr` configs): critic `BaseObstacle` replaced with
  `ObstacleFootprint`. `BaseObstacle` checks only the cell under the robot
  centre, which may get as close as the inscribed radius (0.235 m) to a wall,
  while the front bumper is 0.28 m from the centre. This is not Jazzy
  specific.

### slam_toolbox (`src/robocup_sim/launch/robocup.launch.py`)

slam_toolbox is a lifecycle node in Jazzy. Started as a plain `Node` it stays
`unconfigured` and publishes neither `/map` nor `map -> odom`. It is now a
`LifecycleNode` that the launch file configures and activates on start (same
as slam_toolbox's own `online_async_launch.py`). `package.xml` got
`launch_ros` and `lifecycle_msgs` as dependencies.

### Known issues

- With `--nav` the robot still sometimes drives into walls; not solved yet.
- `/scan` arrives at about 6 Hz instead of 10 Hz on an RTX 3060 Laptop,
  because the simulation runs slower than real time.
