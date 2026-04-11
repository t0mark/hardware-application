import os
import threading

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    MoveItErrorCodes, RobotState, MotionPlanRequest,
    Constraints, JointConstraint, PlanningOptions,
)
from moveit_msgs.srv import GetCartesianPath
from std_msgs.msg import String


class BehaviorExecutorNode(Node):
    def __init__(self):
        super().__init__('behavior_executor')

        self.declare_parameter('behavior_dir', '')
        self.declare_parameter('command_topic', '/p_zone/command')
        self.declare_parameter('min_path_fraction', 0.9)

        behavior_dir_param = self.get_parameter('behavior_dir').value
        command_topic = self.get_parameter('command_topic').value
        self.min_fraction = self.get_parameter('min_path_fraction').value

        if behavior_dir_param:
            self.behavior_dir = behavior_dir_param
        else:
            pkg_share = get_package_share_directory('p_zone')
            self.behavior_dir = os.path.join(pkg_share, 'config', 'behavior')

        self._is_executing = False

        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            String, command_topic, self._command_cb, 10,
            callback_group=cb_group,
        )

        self._cartesian_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path',
            callback_group=cb_group,
        )
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
            callback_group=cb_group,
        )
        self._move_group_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=cb_group,
        )

        self.get_logger().info(
            f'Behavior Executor ready. behavior_dir={self.behavior_dir}'
        )

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def _command_cb(self, msg: String):
        command = msg.data.strip()
        self.get_logger().info(f'Command received: "{command}"')

        if self._is_executing:
            self.get_logger().warn('Already executing, ignoring command')
            return

        yaml_path = os.path.join(self.behavior_dir, f'{command}.yaml')
        if not os.path.exists(yaml_path):
            self.get_logger().error(f'Behavior file not found: {yaml_path}')
            return

        with open(yaml_path, 'r') as f:
            behavior = yaml.safe_load(f)

        thread = threading.Thread(target=self._run_behavior, args=(behavior,), daemon=True)
        thread.start()

    # ------------------------------------------------------------------ #
    #  Dispatch                                                            #
    # ------------------------------------------------------------------ #

    def _run_behavior(self, behavior: dict):
        motion_type = behavior.get('motion_type', 'cartesian')
        if motion_type == 'joint':
            self._run_joint_behavior(behavior)
        else:
            self._run_cartesian_behavior(behavior)

    # ------------------------------------------------------------------ #
    #  Joint motion                                                        #
    # ------------------------------------------------------------------ #

    def _run_joint_behavior(self, behavior: dict):
        waypoints = behavior.get('waypoints', [])
        if not waypoints:
            self.get_logger().warn('No waypoints in behavior file')
            return

        group_name = behavior.get('group_name', 'mainpulation')
        joint_names = ['base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']

        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            return

        self._is_executing = True
        try:
            for i, wp in enumerate(waypoints):
                self.get_logger().info(f'Moving to joint waypoint {i + 1}/{len(waypoints)}...')

                constraints = Constraints()
                for name in joint_names:
                    jc = JointConstraint()
                    jc.joint_name = name
                    jc.position = float(wp[name])
                    jc.tolerance_above = 0.01
                    jc.tolerance_below = 0.01
                    jc.weight = 1.0
                    constraints.joint_constraints.append(jc)

                plan_req = MotionPlanRequest()
                plan_req.group_name = group_name
                plan_req.goal_constraints = [constraints]
                plan_req.num_planning_attempts = 10
                plan_req.allowed_planning_time = 5.0
                plan_req.max_velocity_scaling_factor = 0.1
                plan_req.max_acceleration_scaling_factor = 0.1

                planning_options = PlanningOptions()
                planning_options.plan_only = False
                planning_options.replan = False

                goal = MoveGroup.Goal()
                goal.request = plan_req
                goal.planning_options = planning_options

                send_future = self._move_group_client.send_goal_async(goal)
                done = threading.Event()
                send_future.add_done_callback(lambda _: done.set())
                if not done.wait(timeout=10.0):
                    self.get_logger().error(f'Waypoint {i + 1} send goal timed out')
                    return

                goal_handle = send_future.result()
                if not goal_handle.accepted:
                    self.get_logger().error(f'Waypoint {i + 1} goal rejected')
                    return

                result_future = goal_handle.get_result_async()
                done = threading.Event()
                result_future.add_done_callback(lambda _: done.set())
                if not done.wait(timeout=60.0):
                    self.get_logger().error(f'Waypoint {i + 1} execution timed out')
                    return

                result = result_future.result().result
                if result.error_code.val != MoveItErrorCodes.SUCCESS:
                    self.get_logger().error(
                        f'Waypoint {i + 1} failed, error_code={result.error_code.val}'
                    )
                    return

                self.get_logger().info(f'Waypoint {i + 1} reached')
        finally:
            self._is_executing = False

    # ------------------------------------------------------------------ #
    #  Cartesian motion                                                    #
    # ------------------------------------------------------------------ #

    def _run_cartesian_behavior(self, behavior: dict):
        poses = self._parse_poses(behavior.get('poses', []))
        if not poses:
            self.get_logger().warn('No poses in behavior file')
            return

        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('compute_cartesian_path service not available')
            return

        req = GetCartesianPath.Request()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = behavior.get('frame_id', 'link0')
        req.group_name = behavior.get('group_name', 'mainpulation')
        req.link_name = behavior.get('end_effector_link', 'tcp')
        req.waypoints = poses
        req.max_step = float(behavior.get('max_step', 0.01))
        req.jump_threshold = float(behavior.get('jump_threshold', 0.0))
        req.avoid_collisions = bool(behavior.get('avoid_collisions', True))

        req.start_state = RobotState()
        req.start_state.is_diff = True

        self.get_logger().info(f'Computing Cartesian path for {len(poses)} waypoints...')
        future = self._cartesian_client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=15.0):
            self.get_logger().error('Cartesian path computation timed out')
            return

        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f'Cartesian path failed, error_code={response.error_code.val}'
            )
            return

        fraction = response.fraction
        self.get_logger().info(f'Cartesian path computed, fraction={fraction:.3f}')

        if fraction < self.min_fraction:
            self.get_logger().error(
                f'Path fraction {fraction:.3f} < minimum {self.min_fraction}, aborting'
            )
            return

        self._execute_trajectory(response.solution)

    def _execute_trajectory(self, trajectory):
        if not self._execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('ExecuteTrajectory action server not available')
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self._is_executing = True
        self.get_logger().info('Executing trajectory...')

        send_future = self._execute_client.send_goal_async(goal)
        done = threading.Event()
        send_future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=10.0):
            self.get_logger().error('Send goal timed out')
            self._is_executing = False
            return

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory execution rejected by server')
            self._is_executing = False
            return

        result_future = goal_handle.get_result_async()
        done = threading.Event()
        result_future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=120.0):
            self.get_logger().error('Trajectory execution timed out')
            self._is_executing = False
            return
        result = result_future.result().result
        self._is_executing = False

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('Trajectory executed successfully')
        else:
            self.get_logger().error(
                f'Trajectory execution failed, error_code={result.error_code.val}'
            )

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _parse_poses(raw_poses: list) -> list[Pose]:
        poses = []
        for p in raw_poses:
            pose = Pose()
            pos = p.get('position', {})
            ori = p.get('orientation', {})
            pose.position.x = float(pos.get('x', 0.0))
            pose.position.y = float(pos.get('y', 0.0))
            pose.position.z = float(pos.get('z', 0.0))
            pose.orientation.x = float(ori.get('x', 0.0))
            pose.orientation.y = float(ori.get('y', 0.0))
            pose.orientation.z = float(ori.get('z', 0.0))
            pose.orientation.w = float(ori.get('w', 1.0))
            poses.append(pose)
        return poses


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
