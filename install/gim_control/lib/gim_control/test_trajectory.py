#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('test_trajectory_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def send_goal(self, positions):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint']
        
        point = JointTrajectoryPoint()
        point.positions = positions
        # Move to target over 2.0 seconds
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        goal_msg.trajectory.points.append(point)
        
        self.get_logger().info(f'Sending trajectory goal: {positions}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result code: {result.error_code}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryActionClient()
    
    target_positions = [0.5, 0.5, 0.5] 
    # Use a timer to delay sending the goal until spin() has started processing events
    action_client.create_timer(1.0, lambda: action_client.send_goal(target_positions))
    
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
