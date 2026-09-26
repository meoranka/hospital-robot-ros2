# hospital-robot-ros2

ROS 2 hospital robot workspace.

## Room navigation (Pavol)

The `navigation_pkg` package accepts hospital room names on
`/hospital/target_room` and navigates to their map coordinates using Nav2.
A new destination replaces the active goal while the robot is moving.

See [navigation setup, room names and testing](src/navigation_pkg/README.md)
for the ROS 2 Jazzy / Gazebo workflow.
