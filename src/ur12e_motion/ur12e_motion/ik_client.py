"""Plan and execute the stationary D-bar feed sequence through MoveIt."""

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
import rclpy
from scipy.spatial.transform import Rotation
from rclpy.action import ActionClient
from rclpy.node import Node

from ur12e_motion.winding_path import generate_winding_sequence


class IKClient(Node):
    """Execute separate wind and transfer segments with manual cut pauses."""

    def __init__(self):
        super().__init__('ik_client')
        self.client = self.create_client(GetPositionIK, '/compute_ik')
        self.cartesian_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path',
        )
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
        )
        self.winding_sequence = generate_winding_sequence()
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('acceleration_scaling', 0.1)
        self.velocity = self.get_parameter('velocity_scaling').value
        self.acceleration = self.get_parameter('acceleration_scaling').value
        if not (0 < self.velocity <= 1 and 0 < self.acceleration <= 1):
            raise ValueError('Velocity and acceleration scaling must be in (0, 1]')

    @staticmethod
    def pose_from_transform(transform):
        """Convert a homogeneous tool transform to a ROS pose."""
        pose = Pose()
        pose.position.x = float(transform[0, 3])
        pose.position.y = float(transform[1, 3])
        pose.position.z = float(transform[2, 3])
        quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
        pose.orientation.x = float(quaternion[0])
        pose.orientation.y = float(quaternion[1])
        pose.orientation.z = float(quaternion[2])
        pose.orientation.w = float(quaternion[3])
        return pose

    def wait_result(self, future):
        """Process ROS callbacks while awaiting a service or action result."""
        rclpy.spin_until_future_complete(self, future)
        if not future.done() or future.cancelled():
            raise RuntimeError('ROS request interrupted')
        return future.result()

    @staticmethod
    def check_success(result, stage='MoveIt request'):
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = result.error_code.val
            names = [name for name in dir(MoveItErrorCodes)
                     if name.isupper() and getattr(MoveItErrorCodes, name) == code]
            name = names[0] if names else 'UNKNOWN'
            detail = result.error_code.message
            raise RuntimeError(f'{stage}: {name} ({code}). {detail}')

    def run_action(self, client, goal):
        handle = self.wait_result(client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('MoveIt rejected the goal')
        try:
            result = self.wait_result(handle.get_result_async())
        except KeyboardInterrupt:
            self.wait_result(handle.cancel_goal_async())
            raise
        if result.status != 4:  # action_msgs/GoalStatus.STATUS_SUCCEEDED
            raise RuntimeError(f'Action ended with status {result.status}')
        self.check_success(result.result)
        return result.result

    def build_request(self, target):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'ur_manipulator'
        request.ik_request.ik_link_name = 'tool0'
        request.ik_request.pose_stamped = target
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 2
        return request

    def approach_start(self):
        """Plan and execute a joint-space approach to the first feed pose."""
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose = self.pose_from_transform(self.winding_sequence[0][1][0])
        p = target.pose.position
        self.get_logger().info(
            f'Approach IK target in base_link: ({p.x:.3f}, {p.y:.3f}, '
            f'{p.z:.3f}) m; tool0 +Z points down.'
        )
        response = self.wait_result(
            self.client.call_async(self.build_request(target)),
        )
        self.check_success(response, 'Approach IK')
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = 'ur_manipulator'
        request.start_state.is_diff = True
        request.num_planning_attempts = 5
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = self.velocity
        request.max_acceleration_scaling_factor = self.acceleration
        constraints = Constraints()
        joints = response.solution.joint_state
        arm_joints = {
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        }
        for name, position in zip(joints.name, joints.position):
            if name in arm_joints:
                constraints.joint_constraints.append(JointConstraint(
                    joint_name=name, position=position,
                    tolerance_above=0.001, tolerance_below=0.001, weight=1.0,
                ))
        if len(constraints.joint_constraints) != 6:
            raise RuntimeError('IK response does not contain the six arm joints')
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        self.run_action(self.move_client, goal)

    def execute_segment(self, poses):
        """Require a complete, timed Cartesian plan before executing."""
        request = GetCartesianPath.Request()
        request.header.frame_id = 'base_link'
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state.is_diff = True
        request.group_name = 'ur_manipulator'
        request.link_name = 'tool0'
        request.waypoints = [self.pose_from_transform(pose) for pose in poses]
        request.max_step = 0.002
        request.revolute_jump_threshold = 0.26
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.velocity
        request.max_acceleration_scaling_factor = self.acceleration
        response = self.wait_result(self.cartesian_client.call_async(request))
        self.check_success(response, 'Cartesian path planning')
        if response.fraction < 0.999999:
            raise RuntimeError(f'Incomplete Cartesian path: {response.fraction:.1%}')
        points = response.solution.joint_trajectory.points
        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                 for p in points]
        if len(times) < 2 or any(b <= a for a, b in zip(times, times[1:])):
            raise RuntimeError('Cartesian trajectory has invalid timing')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        self.run_action(self.execute_client, goal)

    def run_sequence(self):
        for client in (self.client, self.cartesian_client):
            while not client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    return
                self.get_logger().info('Waiting for MoveIt services...')
        for client in (self.move_client, self.execute_client):
            while not client.wait_for_server(timeout_sec=1.0):
                if not rclpy.ok():
                    return
                self.get_logger().info('Waiting for MoveIt actions...')
        input('Press Enter to approach node 1 and wind string 1 (Ctrl+C to stop): ')
        self.approach_start()
        for index, (kind, poses) in enumerate(self.winding_sequence):
            self.get_logger().info(f'Executing segment {index + 1}: {kind}')
            self.execute_segment(poses)
            if kind == 'wind':
                input('String complete. Secure/cut it, then press Enter to continue: ')
        self.get_logger().info('D-bar winding sequence complete.')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IKClient()
        node.run_sequence()
    except (KeyboardInterrupt, EOFError):
        pass
    except (RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().error(f'Sequence stopped: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
