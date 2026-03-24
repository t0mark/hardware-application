import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
)
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg


INITIAL_JOINT_POSITIONS = {
    'joint1': 0.07928763197383908,
    'joint2': -0.2774707592337207,
    'joint3': 0.9991727672591988,
    'joint4': 0.5257000096983714,
    'joint5': 1.524633092459463,
    'joint6': 0.007286408742456795,
}


class InitialPoseNode(Node):
    def __init__(self):
        super().__init__('initial_pose_node')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        joint_constraints = []
        for name, position in INITIAL_JOINT_POSITIONS.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            joint_constraints.append(jc)

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = joint_constraints

        request = MotionPlanRequest()
        request.group_name = 'arm'
        request.goal_constraints = [goal_constraints]
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        options = PlanningOptions()
        options.plan_only = False

        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options = options

        self.get_logger().info('Sending goal...')
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.state}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result error code: {result.error_code.val}')
        self.open_gripper()

    def open_gripper(self):
        self.get_logger().info('Opening gripper...')
        self._gripper_client.wait_for_server()

        gripper_cmd = GripperCommandMsg()
        gripper_cmd.position = 0.0
        gripper_cmd.max_effort = 50.0

        goal_msg = GripperCommand.Goal()
        goal_msg.command = gripper_cmd

        future = self._gripper_client.send_goal_async(goal_msg)
        future.add_done_callback(self.gripper_response_callback)

    def gripper_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            rclpy.shutdown()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future):
        self.get_logger().info('Gripper opened')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseNode()
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
