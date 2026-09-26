"""Offline state-machine tests; not a replacement for a live ROS/Nav2 test."""

import ast
from concurrent.futures import Future
from enum import Enum
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]


class TaskResult(Enum):
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3
    UNKNOWN = 4


def pose_stamped():
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='', stamp=None),
        pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                             orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)))


# Execute the actual pure logic, excluding ROS imports and main/init calls.
tree = ast.parse((ROOT / 'scripts/hospital_navigator.py').read_text())
definitions = [item for item in tree.body
               if isinstance(item, (ast.Assign, ast.ClassDef))
               or isinstance(item, ast.FunctionDef) and item.name == 'read_rooms']
scope = dict(Node=object, Path=Path, yaml=yaml, math=math, json=json,
             PoseStamped=pose_stamped, TaskResult=TaskResult)
exec(compile(ast.Module(body=definitions, type_ignores=[]), '<navigation_logic>', 'exec'), scope)
HospitalNavigator = scope['HospitalNavigator']
read_rooms = scope['read_rooms']


class FakeNavigator:
    def __init__(self):
        self.accept = True
        self.calls = []
        self.available = True
        self.goal_handle = None
        self.result_future = None
        self.completion_checks = 0
        self.nav_to_pose_client = SimpleNamespace(server_is_ready=lambda: self.available)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'ROS_TIME'))

    def goToPose(self, pose):
        self.calls.append(('go', pose))
        # Match Jazzy: the handle changes even when the server rejects a goal;
        # the result future changes only after an accepted goal.
        self.goal_handle = SimpleNamespace(accepted=self.accept)
        if self.accept:
            self.result_future = Future()
        return self.accept

    def isTaskComplete(self):
        self.completion_checks += 1
        return self.result_future.done()

    def getResult(self):
        return self.result_future.result()

    def cancelTask(self):
        self.calls.append(('cancel',))


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.node = HospitalNavigator.__new__(HospitalNavigator)
        self.node.frame_id, self.node.rooms = read_rooms(ROOT / 'config/hospital_rooms.yaml')
        self.node.ready = True
        self.node.active_room = None
        self.node.pending_room = None
        self.events = []
        self.node.report = lambda *args, **kwargs: self.events.append((args, kwargs))
        self.nav = FakeNavigator()

    def send(self, room):
        self.node.on_target(SimpleNamespace(data=room))

    def test_all_eight_goals_have_correct_frame_time_and_quaternion(self):
        for room, (x, y, yaw) in self.node.rooms.items():
            goal = self.node.make_pose(room, self.nav)
            self.assertEqual(goal.header.frame_id, 'map')
            self.assertEqual(goal.header.stamp, 'ROS_TIME')
            self.assertEqual((goal.pose.position.x, goal.pose.position.y), (x, y))
            self.assertAlmostEqual(goal.pose.orientation.w**2 + goal.pose.orientation.z**2, 1.0)
        self.node.rooms['RoomCT'] = (1.0, 2.0, math.pi)
        self.assertAlmostEqual(self.node.make_pose('RoomCT', self.nav).pose.orientation.z, 1.0)

    def test_callback_only_queues_then_main_loop_sends_goal(self):
        self.send(' RoomCT ')
        self.assertEqual(self.nav.calls, [])
        self.node.step(self.nav)
        self.assertEqual(self.node.active_room, 'RoomCT')
        self.assertEqual(self.nav.calls[0][0], 'go')

    def test_unknown_room_preserves_active_and_pending_goal(self):
        self.node.active_room, self.node.pending_room = 'RoomCT', 'RoomTR'
        self.send('CT_Scan')
        self.assertEqual((self.node.active_room, self.node.pending_room), ('RoomCT', 'RoomTR'))
        self.assertEqual(self.events[-1][0][0], 'UNKNOWN_ROOM')

    def test_request_before_ready_does_not_move_robot(self):
        self.node.ready = False
        self.send('RoomCT')
        self.node.step(self.nav)
        self.assertEqual(self.nav.calls, [])

    def test_new_target_is_sent_while_previous_goal_is_still_running(self):
        self.send('RoomCT')
        self.node.step(self.nav)
        first_result = self.nav.result_future
        self.send('RoomTR')
        self.node.step(self.nav)
        self.assertFalse(first_result.done())
        self.assertEqual(self.node.active_room, 'RoomTR')
        self.assertEqual([call[0] for call in self.nav.calls], ['go', 'go'])
        self.assertEqual(self.nav.completion_checks, 0)
        self.assertIn((('SWITCHING', 'RoomTR', 'Previous target: RoomCT'), {}), self.events)

    def test_latest_request_replaces_pending_request(self):
        self.send('RoomCT')
        self.node.step(self.nav)
        self.send('RoomTR')
        self.send('RoomAdmin')
        self.node.step(self.nav)
        self.assertEqual(self.node.active_room, 'RoomAdmin')
        self.assertEqual([call[0] for call in self.nav.calls], ['go', 'go'])
        goal = self.nav.calls[-1][1]
        self.assertEqual((goal.pose.position.x, goal.pose.position.y),
                         self.node.rooms['RoomAdmin'][:2])

    def test_several_preemptions_and_late_old_results_preserve_new_goal(self):
        for old_result in (TaskResult.CANCELED, TaskResult.SUCCEEDED, TaskResult.FAILED):
            self.send('RoomCT')
            self.node.step(self.nav)
            first_result = self.nav.result_future
            self.send('RoomTR')
            self.node.step(self.nav)
            second_result = self.nav.result_future
            self.send('RoomAdmin')
            self.node.step(self.nav)
            current_result = self.nav.result_future
            first_result.set_result(old_result)
            second_result.set_result(old_result)
            self.node.step(self.nav)
            self.assertEqual(self.node.active_room, 'RoomAdmin')
            self.assertIs(self.nav.result_future, current_result)
            self.assertFalse(current_result.done())
            current_result.set_result(TaskResult.SUCCEEDED)
            self.node.step(self.nav)
            self.assertIsNone(self.node.active_room)
            self.assertEqual(self.events[-1][0], ('SUCCEEDED', 'RoomAdmin'))
        self.assertTrue(all(call[0] == 'go' for call in self.nav.calls))

    def test_running_goal_does_not_block_on_result_polling(self):
        self.send('RoomCT')
        self.node.step(self.nav)
        for _ in range(5):
            self.node.step(self.nav)
        self.assertEqual(self.nav.completion_checks, 0)

    def test_request_for_active_room_clears_other_pending_request(self):
        self.send('RoomCT')
        self.node.step(self.nav)
        self.send('RoomAdmin')
        self.send('RoomCT')
        self.node.step(self.nav)
        self.assertIsNone(self.node.pending_room)
        self.assertEqual(self.node.active_room, 'RoomCT')
        self.assertEqual(len(self.nav.calls), 1)

    def test_repeated_active_target_does_not_restart(self):
        self.send('RoomCT')
        self.node.step(self.nav)
        self.send('RoomCT')
        self.node.step(self.nav)
        self.assertEqual(len(self.nav.calls), 1)

    def test_rejected_goal_does_not_become_active(self):
        self.nav.accept = False
        self.send('RoomCT')
        self.node.step(self.nav)
        self.assertIsNone(self.node.active_room)
        self.assertEqual(self.events[-1][0][0], 'REJECTED')

    def test_rejected_replacement_keeps_original_handle_and_result(self):
        self.send('RoomCT')
        self.node.step(self.nav)
        original_handle = self.nav.goal_handle
        original_result = self.nav.result_future
        self.nav.accept = False
        self.send('RoomAdmin')
        self.node.step(self.nav)
        self.assertEqual(self.node.active_room, 'RoomCT')
        self.assertIs(self.nav.goal_handle, original_handle)
        self.assertIs(self.nav.result_future, original_result)
        self.assertEqual(self.events[-1][0][0], 'REJECTED')
        original_result.set_result(TaskResult.SUCCEEDED)
        self.node.step(self.nav)
        self.assertEqual(self.events[-1][0], ('SUCCEEDED', 'RoomCT'))

    def test_missing_nav2_server_does_not_enter_blocking_goal_call(self):
        self.nav.available = False
        self.send('RoomCT')
        self.node.step(self.nav)
        self.assertEqual(self.nav.calls, [])
        self.assertEqual(self.events[-1][0][0], 'NAV2_UNAVAILABLE')

    def test_terminal_results_are_reported(self):
        for result in TaskResult:
            self.send('RoomMain')
            self.node.step(self.nav)
            self.nav.result_future.set_result(result)
            self.node.step(self.nav)
            self.assertIsNone(self.node.active_room)
            expected = 'UNKNOWN_RESULT' if result == TaskResult.UNKNOWN else result.name
            self.assertEqual(self.events[-1][0][0], expected)

    def test_missing_or_nonfinite_coordinates_fail_before_navigation(self):
        config = yaml.safe_load((ROOT / 'config/hospital_rooms.yaml').read_text())
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'rooms.yaml'
            for bad in (None, float('nan'), float('inf'), '2.0', True):
                config['rooms']['RoomCT']['x'] = bad
                path.write_text(yaml.safe_dump(config))
                with self.assertRaises(ValueError):
                    read_rooms(path)


if __name__ == '__main__':
    unittest.main()
