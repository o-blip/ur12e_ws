"""Generate a winding path and solve inverse kinematics for every pose."""

from geometry_msgs.msg import PoseStamped

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK

import numpy as np

import rclpy
from rclpy.node import Node

from ur12e_motion.tenseg_geometry import tensegrity_nodes
from ur12e_motion.winding_path import (
    generate_complete_prism_path,
    homogeneous_transform,
    INCH_TO_METER,
    transform_to_pose,
)


class IKClient(Node):
    """Solve the complete winding path through MoveIt's IK service."""

    def __init__(self):
        """Create the path and begin requesting IK solutions."""

        #region  Boot up client/server 
        # Create a ROS 2 node named "ik_client".
        super().__init__('ik_client')

        # Create a client for MoveIt's inverse-kinematics service.
        self.client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )
        # Wait until MoveIt has started and the service is available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik...')
        #endregion

        #region Define geometry and cartesian trajectory 
        # Define the tensegrity geometry.
        radius = 4.0 * np.sqrt(3.0) * INCH_TO_METER
        height = radius
        nodes = tensegrity_nodes(radius, height)

        # Reference prism setup copied from the UR5e experiment.
        # Confirm the feed position and tool offset for the new fixture.
        # Fixed string feed point in the robot base frame.
        feed_position = np.array([
            20.0 * INCH_TO_METER,
            0.0 * INCH_TO_METER,
            20.0 * INCH_TO_METER,
        ])

        # Fixed transform from tool0 to the prism center.
        T_E_P = homogeneous_transform(
            R=np.eye(3),
            p=np.array([0.0, 0.0, 8.0 * INCH_TO_METER]),
        )

        # Cartesian trajectory
        prism_poses = generate_complete_prism_path(
            nodes,
            feed_position,
        )

        # Transform prism poses to tool0 poses
        # T_B_P = T_B_E @ T_E_P, so T_B_E = T_B_P @ inverse(T_E_P).
        T_P_E = np.linalg.inv(T_E_P)
        self.tool_poses = [
            T_B_P @ T_P_E
            for T_B_P in prism_poses
        ]

        self.get_logger().info(
            f'Generated {len(self.tool_poses)} tool poses'
        )
        #endregion
        
        #region UR12e Robot parameters 
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        # Matches ur_description/config/ur12e/joint_limits.yaml.
        # The elbow uses the upstream +/- pi planning restriction.
        self.lower_joint_limits = np.array([
            -2 * np.pi,  # shoulder_pan_joint
            -2 * np.pi,  # shoulder_lift_joint
            -1 * np.pi,  # elbow_joint
            -2 * np.pi,  # wrist_1_joint
            -2 * np.pi,  # wrist_2_joint
            -2 * np.pi,  # wrist_3_joint
        ])

        self.upper_joint_limits = np.array([
            2 * np.pi,  # shoulder_pan_joint
            2 * np.pi,  # shoulder_lift_joint
            1 * np.pi,  # elbow_joint
            2 * np.pi,  # wrist_1_joint
            2 * np.pi,  # wrist_2_joint
            2 * np.pi,  # wrist_3_joint
        ])
        #endregion

        #region Multi-seed initializing
        self.rng = np.random.default_rng(45) # fixed rng seed for reproducibility
        self.unique_seed_solutions = []
        self.unique_solution_count = 0
        self.seed_max = 8  # Eight possible solutions.
        self.seed_attempts = 0
        self.seed_max_attempts = 100 # stop trying after this many attempts
        self.solution_tolerance = np.deg2rad(5.0) # tolerance for considering two IK solutions as the same
        #endregion

        #region Path-sweep state initialization
        # Path-sweep state. Each entry in joint_paths contains the trajectory
        # generated from one unique first-pose solution.
        self.joint_paths = []
        self.failed_seed_paths = []
        self.seed_index = 0
        self.pose_index = 0
        self.max_joint_step = np.deg2rad(15.0)
        #endregion

        self.request_seed_solution()

    def start_seed_sweep(self):
        """Start trajectory generation from every unique first-pose seed."""
        #region Error handling
        if not self.unique_seed_solutions:
            self.get_logger().error(
                'Cannot start seed sweep: no first-pose IK solutions found.'
            )
            return
        #endregion
        self.seed_index = 0
        # Every stored seed already solves the first tool pose.
        self.pose_index = 1
        self.joint_paths = [
            [np.asarray(seed_solution).copy()]
            for seed_solution in self.unique_seed_solutions
        ]

        self.get_logger().info(
            f'Starting path sweep for {len(self.joint_paths)} seeds.'
        )

        if self.pose_index >= len(self.tool_poses):
            self.advance_to_next_seed(completed=True)
        else:
            self.request_path_solution()

    def handle_path_response(self, future):
        """Store one path solution and request the following pose."""
        #region Error handling
        try:
            response = future.result()
        except Exception as error:
            self.advance_to_next_seed(
                completed=False,
                failure_reason=f'IK service call failed: {error}',
            )
            return

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.advance_to_next_seed(
                completed=False,
                failure_reason=(
                    f'IK failed with error code {response.error_code.val}'
                ),
            )
            return
        #endregion
        joint_names = list(response.solution.joint_state.name)
        joint_positions = np.asarray(
            response.solution.joint_state.position
        )

        # Check if joint names are consistent
        if self.joint_names != joint_names:
            self.advance_to_next_seed(
                completed=False,
                failure_reason=(
                    f'Joint names are inconsistent: '
                    f'{self.joint_names} vs {joint_names}'
                ),
            )
            return

        # Measure the shortest angular change from the preceding pose. This
        # treats equivalent angles such as +pi and -pi as neighboring values.
        previous_positions = self.joint_paths[self.seed_index][-1]
        joint_deltas = self.wrapped_angle_difference(
            previous_positions,
            joint_positions,
        )
        # Check if the joint steps are within the allowed maximum.
        if np.any(np.abs(joint_deltas) > self.max_joint_step):
            max_step = np.max(np.abs(joint_deltas))
            self.advance_to_next_seed(
                completed=False,
                failure_reason=(
                    f'joint discontinuity; maximum step was '
                    f'{np.rad2deg(max_step):.2f} degrees'
                ),
            )
            return

        # Store the equivalent joint angles nearest the preceding positions,
        # preventing artificial 2*pi jumps in the saved trajectory.
        continuous_positions = previous_positions + joint_deltas
        self.joint_paths[self.seed_index].append(continuous_positions)
        self.pose_index += 1

        if self.pose_index < len(self.tool_poses):
            self.request_path_solution()
        else:
            self.advance_to_next_seed(completed=True)

    def request_path_solution(self):
        """Request the next pose, seeded by the preceding joint solution."""
        previous_positions = self.joint_paths[self.seed_index][-1]

        target = self.build_target()
        request = self.build_request(target)

        request.ik_request.robot_state.joint_state.name = self.joint_names
        request.ik_request.robot_state.joint_state.position = (
            previous_positions.tolist()
        )

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_path_response)



    def advance_to_next_seed(self, completed, failure_reason=None):
        """Record the current result and advance to the next seed."""
        #region Completed or failed seed handling
        if completed:
            self.get_logger().info(
                f'Seed {self.seed_index + 1}/{len(self.joint_paths)} '
                f'completed {len(self.joint_paths[self.seed_index])} poses.'
            )
        else:
            self.get_logger().warning(
                f'Seed {self.seed_index + 1}/{len(self.joint_paths)} failed '
                f'at pose {self.pose_index}: {failure_reason}'
            )
            self.failed_seed_paths.append(self.seed_index)
        #endregion

        self.seed_index += 1
        if self.seed_index >= len(self.joint_paths):
            completed_count = (
                len(self.joint_paths) - len(self.failed_seed_paths)
            )
            self.get_logger().info(
                f'Seed sweep complete: {completed_count}/'
                f'{len(self.joint_paths)} paths completed.'
            )
            return

        self.pose_index = 1
        self.request_path_solution()

    def handle_seed_response(self, future):
        """Process one first-pose result from a randomized seed."""
        #region Error handling
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'IK service call failed: {error}'
            )
            self.seed_attempts += 1
            self.request_seed_solution()
            return

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warning(
                f'IK failed for seed {self.unique_solution_count} '
                f'with error code {response.error_code.val}'
            )
            self.seed_attempts += 1
            self.request_seed_solution()
            return
        #endregion

        joint_names = list(response.solution.joint_state.name)
        joint_positions = list(response.solution.joint_state.position)

        # Check if joint names are consistent
        if self.joint_names != joint_names:
            self.get_logger().error(
                f'Joint names are inconsistent: '
                f'{self.joint_names} vs {joint_names}'
            )
            return

        # Store the result only if it represents a new IK solution.
        if self.is_repeat_solution(joint_positions):
            self.get_logger().info(
                f'Seed {self.unique_solution_count} produced a repeat solution.'
            )
            self.seed_attempts += 1
        else:
            self.seed_attempts = 0
            self.unique_solution_count += 1
            self.unique_seed_solutions.append(joint_positions.copy())
            self.get_logger().info(
                f'Seed {self.unique_solution_count} found unique solution '
                f'{len(self.unique_seed_solutions)}.'
            )

        self.request_seed_solution()

    def request_seed_solution(self):
        """Request a first-pose IK solution using a new random seed."""
        # Stop after finding all branches or exhausting repeat attempts.
        attempts_exhausted = self.seed_attempts >= self.seed_max_attempts
        branches_found = self.unique_solution_count >= self.seed_max
        if attempts_exhausted or branches_found:
            self.get_logger().info(
                f'Seed testing complete: found {len(self.unique_seed_solutions)} unique solutions.'
            )
            self.start_seed_sweep() # start the next phase of IK solving
            return

        target = self.build_target()
        request = self.build_request(target)

        # Solve the IK request using the randomized seed
        request.ik_request.robot_state.joint_state.name = (self.joint_names)
        seed = self.generate_seed(
            self.lower_joint_limits,
            self.upper_joint_limits,
        )
        request.ik_request.robot_state.joint_state.position = seed.tolist()

        self.get_logger().info(
            f'Requesting first-pose IK with seed '
            f'{self.unique_solution_count}/{self.seed_max}'
        )

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_seed_response)

    def is_repeat_solution(self, current_solution):
        """Return True if a matching first-pose solution is already stored."""
        current_solution = np.asarray(current_solution)

        for stored_solution in self.unique_seed_solutions:
            differences = self.wrapped_angle_difference(
                current_solution,
                np.asarray(stored_solution),
            )
            if np.all(np.abs(differences) < self.solution_tolerance):
                return True

        return False

    def generate_seed(self, lower_limit, upper_limit):
        """Generate a random seed within the joint limits."""
        return self.rng.uniform(lower_limit, upper_limit)

    def wrapped_angle_difference(self, angle1, angle2):
        """Compute the shortest angular difference between two angles."""
        return (angle2 - angle1 + np.pi) % (2 * np.pi) - np.pi

    def build_target(self):
        """Build a PoseStamped message from position and quaternion."""
        T_B_E = self.tool_poses[self.pose_index]
        position, quaternion = transform_to_pose(T_B_E)

        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.header.stamp = self.get_clock().now().to_msg()

        target.pose.position.x = float(position[0])
        target.pose.position.y = float(position[1])
        target.pose.position.z = float(position[2])

        target.pose.orientation.x = float(quaternion[0])
        target.pose.orientation.y = float(quaternion[1])
        target.pose.orientation.z = float(quaternion[2])
        target.pose.orientation.w = float(quaternion[3])

        return target

    def build_request(self, target):
        """Build a GetPositionIK request for the given target pose."""
        request = GetPositionIK.Request()

        request.ik_request.group_name = 'ur_manipulator'
        request.ik_request.ik_link_name = 'tool0'
        request.ik_request.pose_stamped = target
        request.ik_request.avoid_collisions = False

        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = 100_000_000

        return request


def main(args=None):
    """Run the IK client node."""
    # Start ROS communication.
    rclpy.init(args=args)

    # Construct the node, which immediately sends the IK request.
    node = IKClient()

    # Keep processing ROS events until the response callback shuts it down.
    rclpy.spin(node)

    # Clean up the node after spinning stops.
    node.destroy_node()


if __name__ == '__main__':
    main()
