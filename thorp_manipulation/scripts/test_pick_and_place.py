#!/usr/bin/env python3

"""
Add a table and a cube in front of the robot to the planning scene, pick the cube and place it on another spot.
Objects exist only in the planning scene, so on simulation the gripper closes on nothing.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from thorp_msgs.action import PickupObject, PlaceObject

TABLE_HEIGHT = 0.35
CUBE_SIDE = 0.025


def make_box(name, x, y, z, size):
    obj = CollisionObject(id=name, operation=CollisionObject.ADD)
    obj.header.frame_id = 'base_footprint'
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = x, y, z
    obj.pose.orientation.w = 1.0
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)]
    return obj


class TestPickAndPlace(Node):
    def __init__(self):
        super().__init__('test_pick_and_place')
        self._apply_scene = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self._pickup = ActionClient(self, PickupObject, 'manipulation/pickup_object')
        self._place = ActionClient(self, PlaceObject, 'manipulation/place_object')

    def add_objects(self):
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [
            make_box('table', 0.45, 0.0, TABLE_HEIGHT - 0.01, [0.4, 0.8, 0.02]),
            make_box('cube', 0.28, 0.03, TABLE_HEIGHT + CUBE_SIDE / 2.0, [CUBE_SIDE] * 3)]
        self._apply_scene.wait_for_service()
        future = self._apply_scene.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def run(self, client, goal):
        client.wait_for_server()
        future = client.send_goal_async(
            goal, feedback_callback=lambda msg: self.get_logger().info(f"Stage: {msg.feedback.stage}"))
        rclpy.spin_until_future_complete(self, future)
        if not future.result().accepted:
            self.get_logger().error("Goal rejected")
            return False
        future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, future)
        error = future.result().result.error
        self.get_logger().info(f"Result: {error.code} ({error.text})")
        return error.code == error.SUCCESS

    def pick_and_place(self):
        if not self.add_objects():
            self.get_logger().error("Failed to add objects to the planning scene")
            return
        if not self.run(self._pickup, PickupObject.Goal(object_name='cube', support_surf='table', tightening=0.002)):
            return
        # gripper target: the cube top, plus some clearance over the table
        place_pose = PoseStamped()
        place_pose.header.frame_id = 'base_footprint'
        place_pose.pose.position.x, place_pose.pose.position.y = 0.26, -0.08
        place_pose.pose.position.z = TABLE_HEIGHT + CUBE_SIDE + 0.005
        place_pose.pose.orientation.w = 1.0
        self.run(self._place, PlaceObject.Goal(object_name='cube', support_surf='table', place_pose=place_pose))


def main():
    rclpy.init()
    node = TestPickAndPlace()
    try:
        node.pick_and_place()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
