#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class FollowJointTrajectoryClient(Node):
    def __init__(self, joint_prifix = ""):
        super().__init__('follow_joint_trajectory_client')
        self.joint_prifix = joint_prifix
        # ros2 action send_goal 에서 썼던 서버 이름 그대로
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self.joint_prifix}joint_trajectory_controller/follow_joint_trajectory"
        )

    def send_goal(self, joint_prifix = "left" , position_list : list = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15], duration:Duration = Duration(sec=3, nanosec=0)):
        self.get_logger().info('Waiting for FollowJointTrajectory action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available, sending goal.')

        # Goal 메시지 생성
        goal_msg = FollowJointTrajectory.Goal()

        traj = JointTrajectory()
        traj.joint_names = [
            f"openarm_{self.joint_prifix}joint1",
            f"openarm_{self.joint_prifix}joint2",
            f"openarm_{self.joint_prifix}joint3",
            f"openarm_{self.joint_prifix}joint4",
            f"openarm_{self.joint_prifix}joint5",
            f"openarm_{self.joint_prifix}joint6",
            f"openarm_{self.joint_prifix}joint7",
        ]

        point = JointTrajectoryPoint()
        point.positions = position_list
        point.time_from_start = duration

        traj.points.append(point)
        goal_msg.trajectory = traj

        # 액션 전송
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # 필요 없으면 로그 지워도 됨
        self.get_logger().info(f'Feedback: {feedback_msg.feedback}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected 😢')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted ✅')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FollowJointTrajectoryClient(joint_prifix="right_")
    node.send_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
