#!/usr/bin/env python3
"""Translate hospital room names to Nav2 goals (ROS 2 Jazzy).

Callbacks only record requests. Commander methods run in the main loop because
they spin their own ROS node internally; calling them from a spinning callback
can otherwise produce executor/reentrancy problems.
Each new destination replaces the current NavigateToPose goal immediately.
"""

import json
import math
from pathlib import Path
import signal
import sys

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
import yaml


ROOM_NAMES = (
    'RoomCT', 'RoomMid', 'RoomMain', 'Corridor',
    'RoomTL', 'RoomTR', 'RoomEnter', 'RoomAdmin',
)


def read_rooms(path):
    """Reject incomplete/nonfinite configurations before any robot command."""
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not isinstance(config.get('rooms'), dict):
        raise ValueError('rooms_file must contain a rooms mapping')
    frame = config.get('frame_id', 'map')
    if not isinstance(frame, str) or not frame.strip():
        raise ValueError('frame_id must be a nonempty string')
    rooms = {}
    for name in ROOM_NAMES:
        value = config['rooms'].get(name)
        if not isinstance(value, dict):
            raise ValueError(f'Missing coordinates for {name}')
        coordinates = []
        for key in ('x', 'y', 'yaw'):
            number = value.get(key, 0.0 if key == 'yaw' else None)
            if isinstance(number, bool) or not isinstance(number, (int, float)):
                raise ValueError(f'{name}.{key} must be a number')
            if not math.isfinite(number):
                raise ValueError(f'{name}.{key} must be finite')
            coordinates.append(float(number))
        rooms[name] = tuple(coordinates)
    return frame, rooms


class HospitalNavigator(Node):
    """Receive room names and manage the currently requested Nav2 destination."""

    def __init__(self):
        """Load room coordinates and create the target and status interfaces."""
        super().__init__('hospital_navigator')
        default_file = str(Path(get_package_share_directory('navigation_pkg')) /
                           'config' / 'hospital_rooms.yaml')
        self.declare_parameter('rooms_file', default_file)
        self.declare_parameter('localizer', 'amcl')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.frame_id, self.rooms = read_rooms(self.get_parameter('rooms_file').value)
        self.ready = False
        self.active_room = None
        self.pending_room = None
        self.stop_requested = False
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(String, '/hospital/navigation_status', status_qos)
        self.target_sub = self.create_subscription(
            String, '/hospital/target_room', self.on_target, 1)

    def report(self, status, room='', detail=''):
        """Publish a JSON status and the corresponding log entry."""
        payload = {'status': status, 'room': room, 'detail': detail}
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.get_logger().info(f'{status}: {room} {detail}'.rstrip())

    def on_target(self, message):
        """Record the latest valid destination without waiting for navigation."""
        room = message.data.strip()
        if room not in self.rooms:
            self.report('UNKNOWN_ROOM', room, 'Allowed: ' + ', '.join(ROOM_NAMES))
            return
        if not self.ready:
            self.report('NOT_READY', room, 'Wait for READY, then publish the target again.')
            return
        # Latest valid request wins. Repeated publication of the same active
        # destination does not repeatedly restart the goal.
        if room == self.active_room:
            self.pending_room = None
            self.report('ALREADY_NAVIGATING', room)
            return
        self.pending_room = room
        self.report('TARGET_RECEIVED', room)

    def make_pose(self, room, navigator):
        """Build a stamped map pose with the configured position and heading."""
        x, y, yaw = self.rooms[room]
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def step(self, navigator):
        """Send pending destinations first, then inspect completed results."""
        # Process new destinations BEFORE checking completion of the old one.
        # Nav2 preempts NavigateToPose with another NavigateToPose using the
        # same behavior tree. Do not cancel and wait for the old result first.
        if self.pending_room is not None:
            room = self.pending_room
            self.pending_room = None
            if not navigator.nav_to_pose_client.server_is_ready():
                self.report('NAV2_UNAVAILABLE', room, 'Start Nav2, then publish the target again.')
                return
            previous_room = self.active_room
            previous_handle = navigator.goal_handle
            previous_result = navigator.result_future
            if navigator.goToPose(self.make_pose(room, navigator)):
                self.active_room = room
                if previous_room is not None:
                    self.report('SWITCHING', room, 'Previous target: ' + previous_room)
                self.report('NAVIGATING', room)
            else:
                # Jazzy Commander overwrites goal_handle even on rejection.
                # Preserve tracking and shutdown cancellation of the old goal.
                navigator.goal_handle = previous_handle
                navigator.result_future = previous_result
                self.report('REJECTED', room, 'Nav2 did not accept the goal.')
            return

        # The main loop spins both ROS nodes. Only inspect an already completed
        # result here so result polling cannot delay incoming room messages.
        if (self.active_room is not None
                and navigator.result_future is not None
                and navigator.result_future.done()
                and navigator.isTaskComplete()):
            result = navigator.getResult()
            status = {
                TaskResult.SUCCEEDED: 'SUCCEEDED',
                TaskResult.CANCELED: 'CANCELED',
                TaskResult.FAILED: 'FAILED',
            }.get(result, 'UNKNOWN_RESULT')
            self.report(status, self.active_room)
            self.active_room = None

    def request_stop(self, _signum, _frame):
        """Interrupt startup or action waits while keeping cancellation possible."""
        self.stop_requested = True
        # Interrupt Commander startup/action waits while keeping the ROS
        # context alive long enough for a best-effort cancellation in finally.
        raise KeyboardInterrupt


def main(args=None):
    """Run the subscriber and Commander until a stop signal or ROS shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    navigator = None
    input_executor = None
    exit_code = 0
    try:
        node = HospitalNavigator()
        # Keep the subscriber on its own executor. Alternating spin_once calls
        # with different timeouts on the shared global executor can repeatedly
        # rebuild its callback iterator, delaying callbacks behind /clock.
        input_executor = SingleThreadedExecutor()
        input_executor.add_node(node)
        signal.signal(signal.SIGINT, node.request_stop)
        signal.signal(signal.SIGTERM, node.request_stop)
        navigator = BasicNavigator(node_name='hospital_nav2_client')
        navigator.set_parameters([Parameter(
            'use_sim_time', value=node.get_parameter('use_sim_time').value)])
        localizer = node.get_parameter('localizer').value
        if localizer not in ('amcl', 'slam_toolbox'):
            raise ValueError('localizer must be amcl or slam_toolbox')

        if localizer == 'amcl':
            node.report('WAITING_LOCALIZATION',
                        detail='Set 2D Pose Estimate in RViz on the hospital map.')
            # Do not let waitUntilNav2Active publish its default zero pose.
            # First receive a real AMCL estimate, which may already be latched.
            while rclpy.ok() and not navigator.initial_pose_received:
                input_executor.spin_once(timeout_sec=0.05)
                rclpy.spin_once(navigator, timeout_sec=0.10)

        node.report('WAITING_NAV2', detail='Waiting for active Nav2 lifecycle nodes.')
        navigator.waitUntilNav2Active(localizer=localizer)
        node.ready = True
        node.report(
            'READY', detail='Listening on /hospital/target_room; immediate goal switching enabled')
        while rclpy.ok() and not node.stop_requested:
            input_executor.spin_once(timeout_sec=0.05)
            rclpy.spin_once(navigator, timeout_sec=0.0)
            node.step(navigator)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        exit_code = 1
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'hospital_navigator: {error}', file=sys.stderr)
    finally:
        if input_executor is not None:
            input_executor.shutdown()
        if navigator is not None:
            # No lifecycleShutdown: this node does not own the shared Nav2 stack.
            if (rclpy.ok() and navigator.goal_handle is not None
                    and navigator.goal_handle.accepted):
                try:
                    # Bounded cancellation, so shutdown still exits if Nav2 died.
                    future = navigator.goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(navigator, future, timeout_sec=2.0)
                except Exception:
                    pass
            navigator.destroy_node()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
