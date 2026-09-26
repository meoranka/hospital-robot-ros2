# Hospital room navigation

ROS 2 **Jazzy** on Ubuntu 24.04. The package uses `ament_cmake` and installs
the Python executable `hospital_navigator` through CMake.

The node subscribes to `/hospital/target_room` (`std_msgs/msg/String`), looks up
the room in `config/hospital_rooms.yaml`, and sends a `PoseStamped` in the `map`
frame through `BasicNavigator.goToPose()`.

## Build

With ROS 2 Jazzy and rosdep already installed, run from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-minimal-tb3-sim ros-jazzy-nav2-simple-commander
rosdep install --from-paths src/navigation_pkg --ignore-src -r -y
colcon build --symlink-install --packages-select navigation_pkg
source install/setup.bash
```

## Start the hospital simulation

The repository contains `models/hospital.glb`. Generate the world using the
same mesh placement used to build the hospital map:

```bash
python3 scripts/restore_hospital_world.py --workspace "$PWD"
```

This creates `worlds/hospital.sdf` with a mesh URI for this checkout. The file
is ignored by Git because the absolute path differs between machines. Existing
different worlds are backed up before replacement. Repeat this command if the
checkout is moved. The world is stored in the project, not in `/tmp`.

In terminal 1, from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py \
  slam:=False headless:=False use_sim_time:=True \
  world:="$PWD/worlds/hospital.sdf" \
  map:="$(ros2 pkg prefix --share navigation_pkg)/maps/hospital_map.yaml" \
  x_pose:=0.0 y_pose:=0.0 z_pose:=0.05
```

If Gazebo, AMCL and Nav2 are already running for this hospital map, use that
stack. Run one simulator and one Nav2 stack.

In RViz, set **Fixed Frame** to `map` and use **2D Pose Estimate** to place the
robot at its actual position and heading. Laser points should align with map
walls. Gazebo spawn coordinates and the saved SLAM frame are not necessarily
identical; the navigation node does not overwrite AMCL with a zero pose.

In terminal 2, from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch navigation_pkg hospital_navigator.launch.py use_sim_time:=true
```

Wait for `READY` with `immediate goal switching enabled`. `WAITING_LOCALIZATION`
means AMCL has not supplied a pose; `WAITING_NAV2` means lifecycle activation is
pending. Targets received before readiness are rejected with `NOT_READY`; send
them again after `READY`. Run only one `hospital_navigator` instance.

## Send a destination

In terminal 3, from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic pub --once /hospital/target_room std_msgs/msg/String "{data: 'RoomCT'}"
```

While the robot is still moving, send a different room:

```bash
ros2 topic pub --once /hospital/target_room std_msgs/msg/String "{data: 'RoomAdmin'}"
```

The node reports `SWITCHING` and `NAVIGATING` for `RoomAdmin`. Nav2 replans toward
the new goal without waiting to reach `RoomCT`. The physical response also
depends on planning, control and simulation timing.

The newest pending valid request wins. Repeating the active destination does
not restart navigation. Unknown rooms do not interrupt the current goal. If a
replacement is rejected by Nav2, the node retains tracking of the original
goal. Late results from replaced goals do not complete the current task.

Read status messages (JSON carried in `std_msgs/msg/String`):

```bash
ros2 topic echo /hospital/navigation_status --qos-durability transient_local
```

Example: `{"status": "SUCCEEDED", "room": "RoomAdmin", "detail": ""}`.

## Rooms and parameters

Names are case-sensitive. Coordinates use the supplied `hospital_map.yaml`
and `hospital_map.pgm` (0.05 m/pixel, origin `[-6.952, -5.435, 0]`).

| Room name | X (m) | Y (m) |
| --- | ---: | ---: |
| `RoomCT` | -1.927 | 3.190 |
| `RoomMid` | -3.177 | -0.060 |
| `RoomMain` | -4.927 | -2.860 |
| `Corridor` | 1.073 | 0.040 |
| `RoomTL` | 3.723 | 3.590 |
| `RoomTR` | 5.773 | 3.590 |
| `RoomEnter` | 5.073 | -0.210 |
| `RoomAdmin` | 5.273 | -3.810 |

All configured `yaw` values are 0 radians. Edit the YAML to change destinations,
then rebuild and restart the node. Revalidate all coordinates if the map changes.

Launch parameters:

- `use_sim_time`: defaults to `true` in the launch file.
- `rooms_file`: absolute path to a replacement room YAML file.
- `localizer`: defaults to `amcl`; `slam_toolbox` is also accepted for a running
  SLAM setup, whose room coordinates must match its current map.

The subscriber uses its own executor. Its callback only records requests;
Commander calls run in the main loop. A second `goToPose()` preempts the current
goal of the same action type, and result polling does not wait for completion.
On Ctrl+C the node attempts bounded goal cancellation without shutting down
the shared Nav2 stack.

## Validation

Run the navigation logic tests without ROS (Python and PyYAML are required):

```bash
python3 -m unittest discover -s src/navigation_pkg/test -v
```

In a ROS workspace, the same tests are registered through `ament_cmake_pytest`:

```bash
colcon test --packages-select navigation_pkg
colcon test-result --verbose
```

The offline tests cover goal generation, immediate goal replacement, rapid
target changes, late old results, rejection handling, duplicate and invalid
requests, and terminal results. They use a Commander double, not a ROS action
server. Pavol reported successful navigation to all eight rooms and successful
goal switching during a journey in the hospital simulation on 2026-09-26.

For a fresh setup, repeat the live test above, check `SUCCEEDED` after arrival,
and send an unknown name to verify `UNKNOWN_ROOM`.

Official API references:

- [Nav2 Jazzy Simple Commander](https://docs.nav2.org/jazzy/configuration_and_development/simple_commander_api/simple_commander_api/)
- [Jazzy BasicNavigator implementation](https://api.nav2.org/nav2-jazzy/html/robot__navigator_8py_source.html)
