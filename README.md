# robocup_ws

Isaac Sim simulation of the MRVK robot "shoddy" (4-wheel skid steer) in the
RoboCup@Home HCC arena, with an optional Nav2 navigation stack.
ROS 2 Jazzy · Isaac Sim 6.2 · Ubuntu 24.04

Changes against the original repository (ROS 2 Humble, one robot model with a
2D lidar):

- the setup is ported to ROS 2 Jazzy, see
  [Migration from Humble to Jazzy](#migration-from-humble-to-jazzy);
- a new robot model **v2** (default) built from the current CAD (Fusion) with
  suspension brackets and a sensor tower Livox Horizon + Basler camera + Hesai
  XT16, and simulated Hesai, camera and Livox IMU, see
  [Robot model v2 and its sensors](#robot-model-v2-and-its-sensors). The
  original model is kept as **v1** (`./run.sh --model v1`).

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
sudo apt install python3-rosdep ros-jazzy-libg2o ros-jazzy-pointcloud-to-laserscan
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
./run.sh --model v1             # original robot model (default: v2)
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

Common to both models:

| Topic / frame | Direction | Notes |
|---|---|---|
| `/clock` | Isaac → | simulation time, everything runs with `use_sim_time` |
| `/odom`, TF `odom → base_link` | Isaac → | exact pose from the physics (no drift) |
| `/scan` | → Nav2, slam_toolbox | `sensor_msgs/LaserScan`, source depends on the model (below) |
| `/cmd_vel` | → Isaac | `geometry_msgs/Twist` |
| TF `base_link → …` | robot_state_publisher | from `robocup_description` |
| TF `map → odom`, `/map` | slam_toolbox (`--nav`) | visualisation only |

Model v1 (`robot.urdf.xacro`):

| Topic | Direction | Notes |
|---|---|---|
| `/scan_raw` | Isaac → | RTX lidar, 360°, 0.1°, 10 Hz, range 0.15–4 m, frame `lidar_frame` |
| `/scan` | relay → | `/scan_raw` with no-hit beams turned from 0.0 into `inf` |

Model v2 (`robot_v2.urdf.xacro`, default):

| Topic | Direction | Notes |
|---|---|---|
| `/hesai/points` | Isaac → | `PointCloud2`, Hesai XT16, frame `hesai_frame` |
| `/scan` | pointcloud_to_laserscan → | slice of `/hesai/points` from 5 cm above the floor to 5 cm above the Hesai, 0.18° |
| `/camera/image_raw`, `/camera/camera_info` | Isaac → | Basler, rgb8, 640 × 400 (sensor 1920 × 1200, same field of view), frame `camera_optical_frame` |
| `/livox/imu` | Isaac → | `Imu`, Livox Horizon built-in IMU, frame `livox_frame` |

Nav2 navigates in the `odom` frame with rolling-window costmaps fed by `/scan`;
there is no static map or global localisation.

## Packages

- `src/robocup_sim` – Isaac Sim script (`isaac/robocup_isaac.py`), sensors of
  model v2 (`isaac/robocup_sensors.py`), launch files, Nav2 / slam_toolbox /
  RViz configs, the arena (`worlds/`).
- `src/robocup_description` – URDF/xacro and meshes of shoddy: v1
  `urdf/robot.urdf.xacro` + `meshes/{wheels,lidar}`, v2
  `urdf/robot_v2.urdf.xacro` + `meshes/v2`.
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
- **Lidar (v1)** stands in for the robot's Hokuyo URG-04LX (which scanned 240° with
  1 cm noise; the simulated one scans 360° without noise or motion distortion).
  RTX lidars see rendered geometry, not colliders.
- Footprint for Nav2 is the rectangle over the bumpers and wheels of v1,
  0.56 × 0.47 m (not updated for v2 yet, see Known issues).

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

## Robot model v2 and its sensors

### URDF and meshes

`src/robocup_description/urdf/robot_v2.urdf.xacro` is a new, self-contained
xacro; `robot.urdf.xacro` (v1) and its macros and meshes are unchanged.

- **Source.** Positions and meshes come from the Fusion export of the whole
  robot (`AssemblyMRVK.usdz`, millimetres) and the per-part USD exports (body,
  wheel, suspension bracket, camera, camera holder, Hesai). Every part was
  placed by fitting it onto the assembly (rigid registration, error below
  1 mm) and converted to Collada (`meshes/v2/*.dae`, metres, Z up, colours
  kept as one material per colour), in full CAD detail (~96 000 triangles,
  8.5 MB, the camera alone 76 000 / 7 MB). Each mesh is already expressed in its own
  link frame, so all visuals have a zero origin. The conversion scripts are not part
  of the repository.
- **Masses** are taken over from v1 (body 30 kg, wheels 0.5 kg each); the
  inertias are computed for a box / cylinder of the real dimensions.
- **Corrections of the CAD files:** the camera part is tilted, its pitch was
  measured from its flat faces (exactly 10.0° down) and moved into the joint;
  the right suspension bracket is not a copy of the left one and is taken from
  the assembly; the Livox part file was empty, its mesh is taken from the
  assembly (without materials, coloured dark grey); one degenerate triangle
  ~10 cm below the camera holder was dropped.

TF tree (all frames fixed except the wheels):

```
base_link                     centre of the body box, 0.1698 m above the floor
├─ imu_frame                  (0, 0, 0), position of the IMU not measured yet
├─ suspension_left/right_link (0, ±0.205, -0.060)
│  └─ wheel_*_link            continuous, (±0.151, ±0.0335, 0), r = 0.1098 m
├─ livox_frame                (0.150, 0, 0.1285), centre of the Livox housing
├─ camera_holder_link         (0.150, 0, 0.1705), bottom of the holder
│  └─ camera_link             (+0.026, 0, +0.030), pitched 10° down
│     └─ camera_optical_frame z forward, x right, y down (REP 103)
└─ hesai_frame                (0.146, 0, 0.2737), rotation axis, middle of the window
```

The origins of `livox_frame` and `hesai_frame` are geometric (housing centre,
window centre), not yet checked against the vendors' manuals.

### Sensors in Isaac Sim (`src/robocup_sim/isaac/robocup_sensors.py`)

Sensor poses are read from the URDF, the sensors are children of the
`base_link` prim.

| Sensor | Simulated as | Parameters |
|---|---|---|
| Hesai XT16 | custom RTX rotary lidar | 16 channels +15°…-15° in 2° steps, 360°, 0.18°, 10 Hz, range 0.05–120 m, accuracy 1 cm, 905 nm, single return; firing times approximated |
| Basler a2A1920-160ucPRO | USD camera | sensor 1920 × 1200, pixel 3.45 µm, image rendered at 640 × 400 (see [Performance](#performance)); **focal length 6 mm is an estimate** (lens unknown), `CAMERA_FOCAL_MM` |
| Livox Horizon IMU | Isaac IMU sensor at `livox_frame` | publishes orientation, angular velocity, acceleration with gravity |
| Livox Horizon lidar | **not simulated** | see Known issues |

- `accumulateOutputs` is on, so one `/hesai/points` message holds a full
  revolution instead of the part rendered in one frame.
- `reportRateBaseHz` is not part of the OmniLidar USD schema but the lidar
  profiles use it; it is created on the prim (20 000 = firings per second).
- **Self-occlusion.** The Hesai origin lies inside its own mesh (radius
  ~5 cm). Without help the housing stops the rays: only ±112° in front and
  14 of 16 channels were left. `nearRangeM` / `rangesMinM` only drop close
  points, the rays still start at the origin. The fix is
  `omni:sensor:Core:rangeOffsetM = 0.1`: rays start 10 cm from the origin,
  the housing stays visible in Isaac and RViz, distances are unchanged
  (checked against the walls). Tried without effect: non-visual material
  `single_sided` (also with `/rtx/materialDb/nonVisualMaterialCSV/enabled`),
  primvar `invisibleToSecondaryRays`.
- The Isaac importer names the visual prims after the mesh files
  (`base_link/hesai_xt16`, …), so the sensor prims are named `rtx_hesai`,
  `sensor_camera`, `sensor_livox_imu` to avoid clashes.

### Other changes for v2

| File | Change |
|---|---|
| `run.sh` | `--model v1\|v2` (default `v2`), passed to the Isaac script and the launch file |
| `src/robocup_sim/isaac/robocup_isaac.py` | `--model`; v1 keeps the 2D lidar, v2 creates the sensors from `robocup_sensors.py` |
| `src/robocup_sim/launch/robocup.launch.py` | argument `model` selects the xacro; v1 starts the `/scan_raw` relay, v2 `pointcloud_to_laserscan` (`/hesai/points` → `/scan`); `robot_description` passed as `ParameterValue(..., value_type=str)` |
| `src/robocup_sim/package.xml` | `exec_depend` `pointcloud_to_laserscan` |
| `src/robocup_sim/config/nav2/nav2_params_*.yaml` | removed `sensor_frame: lidar_frame`, the costmaps take the frame from the `/scan` header (works with both models) |
| `src/robocup_sim/config/robocup.rviz` | displays for `/hesai/points` (PointCloud2) and `/camera/image_raw` (Image) |

`ParameterValue(..., value_type=str)`: without it launch tries to parse the
`xacro` output as YAML and fails as soon as the URDF contains `: ` (e.g. in a
comment) with *"Unable to parse the value of parameter robot_description as
yaml"*. The original v1 URDF happened to parse.

## Performance

Measured headless on an RTX 3060 Laptop with `/clock` (simulated seconds per
wall-clock second):

| Variant | Real-time factor |
|---|---|
| v1 | 0.77 |
| v2, camera image 1920 × 1200 | 0.22 |
| v2 without the camera | 0.71 |
| **v2, camera image 640 × 400 (default)** | **0.58** |
| v2 with meshes simplified to ~13k triangles, camera 1920 × 1200 | 0.23 |

The cost is the camera image rendered every frame, not the meshes (simplified
meshes made no difference). The image is therefore rendered at 640 × 400,
`CAMERA_RESOLUTION` in `src/robocup_sim/isaac/robocup_sensors.py`; the field
of view stays the same because it follows from the sensor size
(`CAMERA_SENSOR_PX`, 1920 × 1200 × 3.45 µm), so only the image gets smaller
(fx 579.7 px instead of 1739.1 px). For the full Basler resolution set
`CAMERA_RESOLUTION = CAMERA_SENSOR_PX` (keep the 16:10 ratio for other sizes).

## Known issues

- With `--nav` the robot still sometimes drives into walls; not solved yet.
- **Nav2 footprint is still the v1 one** (0.56 × 0.47 m). v2 measures about
  0.52 × 0.55 m over the wheels (CAD), i.e. it is ~8 cm wider than the
  footprint. Not changed yet.
- **Livox Horizon lidar is not simulated.** A solid-state RTX lidar with custom
  rays (FoV 81.7° × 25.1°, 24 000 points per frame) publishes no data in Isaac
  Sim 6.2; `Example_Solid_State` does, but stops as soon as its ray angles are
  overwritten. Only the Livox mesh and its IMU are in the simulation.
- The simulation runs slower than real time on an RTX 3060 Laptop (see
  [Performance](#performance)); the camera and IMU rates are far
  below the real sensors (160 fps / 200 Hz), the Hesai gives 10 Hz in
  simulation time. The CPU governor was `powersave`, which Isaac warns about
  at startup; physics runs on the CPU.
- The Hesai is 0.44 m above the floor and its lowest channel points 15° down,
  so low obstacles closer than ~1.6 m are not seen (as on the real robot).
- The scripted Isaac app (`SimulationApp`, `isaacsim.exp.base.python.kit`)
  loads fewer extensions than the full Isaac Sim, e.g. the Action Graph editor
  has to be enabled in *Window → Extensions* (`omni.graph.window.action`).

