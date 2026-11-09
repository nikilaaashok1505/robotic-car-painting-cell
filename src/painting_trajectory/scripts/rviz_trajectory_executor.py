#!/usr/bin/env python3

import asyncio
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6"
]


def to_seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


class TrajectoryExecutor(Node):

    def __init__(self):
        super().__init__("rviz_trajectory_executor")

        self.positions = [0.0] * 6

        self.publisher = self.create_publisher(
            JointState, "/joint_states", 10
        )

        self.timer = self.create_timer(
            0.02, self.publish_state
        )

        self.server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.publish_state()

        self.get_logger().info("RViz trajectory executor started.")
        self.get_logger().info("Publishing /joint_states")
        self.get_logger().info(
            "Action: /arm_controller/follow_joint_trajectory"
        )

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = list(self.positions)
        self.publisher.publish(msg)

    def goal_callback(self, goal):
        names = list(goal.trajectory.joint_names)

        if set(names) != set(JOINTS):
            self.get_logger().error("Invalid joint names.")
            return GoalResponse.REJECT

        if not goal.trajectory.points:
            return GoalResponse.REJECT

        if any(len(p.positions) != len(names)
               for p in goal.trajectory.points):
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute(self, goal_handle):

        result = FollowJointTrajectory.Result()

        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)

        order = [names.index(j) for j in JOINTS]

        points = []

        for p in trajectory.points:
            positions = [p.positions[i] for i in order]
            points.append(
                (to_seconds(p.time_from_start), positions)
            )

        if any(points[i][0] < points[i-1][0]
               for i in range(1, len(points))):

            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            return result

        if points[0][0] > 0:
            points.insert(0, (0.0, list(self.positions)))

        self.get_logger().info(
            f"Executing {len(trajectory.points)} trajectory points."
        )

        start = time.monotonic()
        final_time = points[-1][0]

        while rclpy.ok():

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            elapsed = time.monotonic() - start

            if elapsed >= final_time:
                self.positions = list(points[-1][1])
                self.publish_state()
                break

            index = 0

            while (
                index < len(points)-1
                and elapsed > points[index+1][0]
            ):
                index += 1

            t0, p0 = points[index]
            t1, p1 = points[min(index+1, len(points)-1)]

            alpha = (
                1.0 if t1 <= t0
                else max(0.0, min(1.0, (elapsed-t0)/(t1-t0)))
            )

            self.positions = [
                a + alpha*(b-a)
                for a, b in zip(p0, p1)
            ]

            self.publish_state()

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = JOINTS

            desired = JointTrajectoryPoint()
            desired.positions = list(self.positions)

            actual = JointTrajectoryPoint()
            actual.positions = list(self.positions)

            error = JointTrajectoryPoint()
            error.positions = [0.0] * 6

            feedback.desired = desired
            feedback.actual = actual
            feedback.error = error

            goal_handle.publish_feedback(feedback)

            await asyncio.sleep(0.02)

        goal_handle.succeed()

        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Kinematic execution completed."

        self.get_logger().info("Trajectory execution completed.")

        return result


def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
