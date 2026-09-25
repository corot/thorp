#!/usr/bin/env python3

"""
Gripper command action server for Thorp's one-sided gripper: a single servo opens or closes one finger
to achieve the requested opening, in meters. The action succeeds when the gripper reaches the goal opening
or stalls, what probably means that it's grasping something.

Port of the gripper controller of arbotix_controllers (one-side model only), as modified on
https://github.com/corot/arbotix_ros, thorp branch. BSD license, Copyright (c) 2011-2014 Vanadium Labs LLC.
"""

import collections
import threading
from math import asin, sin

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class OneSideGripperModel:
    """ Simplest of grippers, one servo opens or closes to achieve a particular size opening. """

    def __init__(self, node):
        self.pad_width = node.declare_parameter('pad_width', 0.01).value
        self.finger_length = node.declare_parameter('finger_length', 0.02).value
        self.min_opening = node.declare_parameter('min_opening', 0.0).value
        self.max_opening = node.declare_parameter('max_opening', 0.09).value
        self.center = node.declare_parameter('center', 0.0).value
        self.invert = node.declare_parameter('invert', False).value
        self.joint = node.declare_parameter('joint', 'gripper_joint').value
        command_topic = node.declare_parameter('command_topic', 'gripper_joint_controller/commands').value
        self.logger = node.get_logger()

        # the joint position controller takes a one-element array with the joint angle
        self.pub = node.create_publisher(Float64MultiArray, command_topic, 5)

    def set_command(self, command):
        """ Take an input command of width to open gripper. """
        # check opening limits and bound if necessary
        if command.position > self.max_opening:
            self.logger.warn(f'Command ({command.position:f}) exceeds opening limits; '
                             f'we will use max opening instead ({self.max_opening:f})')
            command.position = self.max_opening
        elif command.position < self.min_opening:
            self.logger.warn(f'Command ({command.position:f}) exceeds opening limits; '
                             f'we will use min opening instead ({self.min_opening:f})')
            command.position = self.min_opening

        # compute angle
        angle = asin((command.position - self.pad_width) / (2 * self.finger_length))
        # publish message
        if self.invert:
            self.pub.publish(Float64MultiArray(data=[-angle + self.center]))
        else:
            self.pub.publish(Float64MultiArray(data=[angle + self.center]))
        return True

    def get_position(self, joint_angle):
        # revert angle calculation done in set_command
        if self.invert:
            angle = self.center - joint_angle
        else:
            angle = joint_angle - self.center

        return sin(angle) * 2 * self.finger_length + self.pad_width


class GripperActionController(Node):
    """ The actual action callbacks. """

    # auxiliary constants; make parameters if not generic enough
    EPSILON_OPENING_DIFF = 0.001  # one millimeter; close enough to goal position
    STATE_HZ_BUFFER_SIZE = 10     # buffer used to estimate gripper joint state topic frequency

    def __init__(self):
        super().__init__('gripper_controller')

        self.state_cb_event = threading.Event()
        self.state_cb_times = collections.deque(maxlen=self.STATE_HZ_BUFFER_SIZE)
        self.current_position = 0.0
        self.current_effort = 0.0

        # time the controller will wait before deciding that the gripper is stalled
        # WARN: if too long, and the commanded pose is smaller than the grasped object,
        # the servo can get jammed (it will stop working ant its led will start blinking)
        self.stalled_time = self.declare_parameter('stalled_time', 0.2).value

        # setup model; only the one-sided gripper is supported
        model = self.declare_parameter('model', 'singlesided').value
        if model != 'singlesided':
            raise ValueError(f'Gripper Controller: unsupported model {model}')
        self.model = OneSideGripperModel(self)

        callback_group = ReentrantCallbackGroup()

        # subscribe to joint_states topic
        self.create_subscription(JointState, 'joint_states', self.state_cb, 10, callback_group=callback_group)

        # create gripper command action server; goals are rejected until we receive joint states
        self.server = ActionServer(self, GripperCommand, '~/gripper_action', self.action_cb,
                                   goal_callback=self.goal_cb, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=callback_group)

    def goal_cb(self, goal):
        if len(self.state_cb_times) < self.state_cb_times.maxlen:
            self.get_logger().error('Gripper Controller: no messages from joint_states topic received')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def action_cb(self, goal_handle):
        """ Take an input command of width to open gripper. """
        command = goal_handle.request.command
        result = GripperCommand.Result()
        feedback = GripperCommand.Feedback()

        self.get_logger().info(f'Gripper Controller action goal received: position {command.position:f}, '
                               f'max_effort {command.max_effort:f}')

        # send command to gripper
        if not self.model.set_command(command):
            goal_handle.abort()
            self.get_logger().info('Gripper Controller: Aborted.')
            return result

        # register progress so we can guess if the gripper is stalled; our buffer
        # must contain up to: stalled_time / joint_states period position values
        times = self.state_cb_times
        T = sum(times[i] - times[i - 1] for i in range(1, len(times) - 1)) / (len(times) - 1)
        progress = collections.deque(maxlen=round(self.stalled_time / T))

        diff_at_start = round(abs(command.position - self.current_position), 3)

        # keep watching for gripper position...
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Gripper Controller: Preempted.')
                return result

            # synchronize with the joints state callbacks; time out to check for cancellation
            if not self.state_cb_event.wait(1.0):
                continue
            self.state_cb_event.clear()

            # break when we have reached the goal position...
            diff = abs(command.position - self.current_position)
            if diff < self.EPSILON_OPENING_DIFF:
                result.reached_goal = True
                break

            # ...or when the gripper is exerting beyond max effort
            if command.max_effort and self.current_effort >= command.max_effort:
                # stop moving to prevent damaging the gripper
                command.position = self.current_position
                self.model.set_command(command)
                self.get_logger().error(f'Gripper Controller: max effort reached at position '
                                        f'{self.current_position:f} ({self.current_effort:f} >= '
                                        f'{command.max_effort:f})')
                result.stalled = True
                break

            # ...or when progress stagnates, probably signaling that the gripper is exerting max effort and not moving
            progress.append(round(diff, 3))  # round to millimeter to neglect tiny motions of the stalled gripper
            if len(progress) == progress.maxlen and progress.count(progress[0]) == len(progress):
                if progress[0] == diff_at_start:
                    # we didn't move at all! is the gripper connected?
                    goal_handle.abort()
                    self.get_logger().error('Gripper Controller: gripper not moving; is the servo connected?')
                    return result

                # buffer full with all-equal positions -> gripper stalled
                command.position = self.current_position
                self.model.set_command(command)
                self.get_logger().error(f'Gripper Controller: gripper stalled at position {self.current_position:f}')
                result.stalled = True
                break

            # publish feedback
            feedback.position = self.current_position
            feedback.effort = self.current_effort
            goal_handle.publish_feedback(feedback)

        # publish one last feedback and the result (identical)
        feedback.position = self.current_position
        feedback.effort = self.current_effort
        feedback.reached_goal = result.reached_goal
        feedback.stalled = result.stalled
        goal_handle.publish_feedback(feedback)

        result.position = self.current_position
        result.effort = self.current_effort
        goal_handle.succeed()
        self.get_logger().info(f'Gripper Controller: Succeeded '
                               f'({"goal reached" if result.reached_goal else "stalled"})')
        return result

    def state_cb(self, joint_states):
        try:
            index = joint_states.name.index(self.model.joint)
            angle = joint_states.position[index]
            effort = joint_states.effort[index] if joint_states.effort else 0.0
        except ValueError:
            # no problem; probably a joint states message unrelated to the gripper
            return

        self.current_position = self.model.get_position(angle)
        self.current_effort = abs(effort)

        self.state_cb_times.append(self.get_clock().now().nanoseconds * 1e-9)

        # notice the action server goal callback that new data is available
        self.state_cb_event.set()


def main():
    rclpy.init()
    node = GripperActionController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
