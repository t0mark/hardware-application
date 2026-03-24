#!/usr/bin/env python3
#
# Block grasp node: receives a Trigger service call, then executes
#   1. Open gripper
#   2. Move arm to pre-grasp  (block_xy, block_z + PRE_GRASP_Z_OFFSET)
#   3. Move arm to grasp pose (block_xy, block_z + GRASP_Z_OFFSET)
#   4. Close gripper
#   5. Return to init pose
#
# Services:
#   /block_grasp/grasp  (std_srvs/Trigger)
#
# Subscribes:
#   /block_detector/block_pose  (geometry_msgs/PoseStamped, link0 frame)

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose, Point
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    PositionConstraint,
    BoundingVolume,
)
from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger


# ── Constants ────────────────────────────────────────────────────────────────
BASE_FRAME = 'link0'
PLANNING_GROUP = 'arm'
EEF_LINK = 'end_effector_link'

PRE_GRASP_Z_OFFSET = 0.15   # m above block
GRASP_Z_OFFSET = 0.02       # m above block centre

GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 1.12
GRIPPER_MAX_EFFORT = 50.0

INIT_JOINT_POSITIONS = {
    'joint1': 0.07928763197383908,
    'joint2': -0.2774707592337207,
    'joint3': 0.9991727672591988,
    'joint4': 0.5257000096983714,
    'joint5': 1.524633092459463,
    'joint6': 0.007286408742456795,
}


class BlockGraspNode(Node):

    def __init__(self) -> None:
        super().__init__('block_grasp_node')

        self._arm_client = ActionClient(self, MoveGroup, '/move_action')
        self._gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )

        self._block_pose: PoseStamped | None = None
        self._busy = False

        self.create_subscription(
            PoseStamped, '/block_detector/pose', self._pose_cb, 10
        )
        self.create_service(Trigger, '/block_grasp/grasp', self._grasp_srv_cb)

        self.get_logger().info(
            'BlockGraspNode ready. Call /block_grasp/grasp to start.'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._block_pose = msg

    def _grasp_srv_cb(self, request, response):
        if self._busy:
            response.success = False
            response.message = 'Already executing a grasp sequence'
            return response
        if self._block_pose is None:
            response.success = False
            response.message = 'No block pose received yet'
            return response

        self._busy = True
        bx = self._block_pose.pose.position.x
        by = self._block_pose.pose.position.y
        bz = self._block_pose.pose.position.z
        self.get_logger().info(
            f'Grasp triggered. Block at ({bx:.3f}, {by:.3f}, {bz:.3f})'
        )

        self._open_gripper(bx, by, bz)

        response.success = True
        response.message = 'Grasp sequence started'
        return response

    # ── Grasp sequence ───────────────────────────────────────────────────────

    def _open_gripper(self, bx, by, bz) -> None:
        self.get_logger().info('Step 1: Opening gripper')
        self._send_gripper(
            GRIPPER_OPEN,
            done_cb=lambda: self._move_pre_grasp(bx, by, bz),
        )

    def _move_pre_grasp(self, bx, by, bz) -> None:
        self.get_logger().info('Step 2: Moving to pre-grasp')
        goal = self._make_position_goal(bx, by, bz + PRE_GRASP_Z_OFFSET)
        self._send_arm(goal, done_cb=lambda: self._move_grasp(bx, by, bz))

    def _move_grasp(self, bx, by, bz) -> None:
        self.get_logger().info('Step 3: Moving to grasp position')
        goal = self._make_position_goal(bx, by, bz + GRASP_Z_OFFSET)
        self._send_arm(goal, done_cb=lambda: self._close_gripper())

    def _close_gripper(self) -> None:
        self.get_logger().info('Step 4: Closing gripper')
        self._send_gripper(GRIPPER_CLOSE, done_cb=self._return_init)

    def _return_init(self) -> None:
        self.get_logger().info('Step 5: Returning to init pose')
        goal = self._make_init_goal()
        self._send_arm(goal, done_cb=self._grasp_done)

    def _grasp_done(self) -> None:
        self.get_logger().info('Grasp sequence complete.')
        self._busy = False

    # ── Goal builders ────────────────────────────────────────────────────────

    def _make_position_goal(self, x: float, y: float, z: float) -> MoveGroup.Goal:
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0

        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [target_pose]

        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = EEF_LINK
        pc.constraint_region = bv
        pc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [pc]

        request = MotionPlanRequest()
        request.group_name = PLANNING_GROUP
        request.goal_constraints = [constraints]
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2

        options = PlanningOptions()
        options.plan_only = False

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options
        return goal

    def _make_init_goal(self) -> MoveGroup.Goal:
        joint_constraints = []
        for name, position in INIT_JOINT_POSITIONS.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            joint_constraints.append(jc)

        constraints = Constraints()
        constraints.joint_constraints = joint_constraints

        request = MotionPlanRequest()
        request.group_name = PLANNING_GROUP
        request.goal_constraints = [constraints]
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2

        options = PlanningOptions()
        options.plan_only = False

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options
        return goal

    # ── Action senders ───────────────────────────────────────────────────────

    def _send_arm(self, goal: MoveGroup.Goal, done_cb) -> None:
        self._arm_client.wait_for_server()
        future = self._arm_client.send_goal_async(goal)

        def _on_goal(f):
            handle = f.result()
            if not handle.accepted:
                self.get_logger().error('Arm goal rejected')
                self._busy = False
                return
            handle.get_result_async().add_done_callback(_on_result)

        def _on_result(f):
            result = f.result().result
            if result.error_code.val != 1:
                self.get_logger().error(
                    f'Arm motion failed (error_code={result.error_code.val})'
                )
                self._busy = False
                return
            done_cb()

        future.add_done_callback(_on_goal)

    def _send_gripper(self, position: float, done_cb) -> None:
        self._gripper_client.wait_for_server()
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = GRIPPER_MAX_EFFORT
        future = self._gripper_client.send_goal_async(goal)

        def _on_goal(f):
            handle = f.result()
            if not handle.accepted:
                self.get_logger().error('Gripper goal rejected')
                self._busy = False
                return
            handle.get_result_async().add_done_callback(lambda _: done_cb())

        future.add_done_callback(_on_goal)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlockGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
