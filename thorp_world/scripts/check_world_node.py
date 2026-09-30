#!/usr/bin/env python3
"""
Exercise a running thorp_world node: observe in base_footprint, update, query relations, save and load. It expects
the robot at (1, 2) on the map facing +y, and the arm base 0.1 m ahead of it; on a ROS domain nothing else uses:

    export ROS_DOMAIN_ID=61 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ros2 node list --no-daemon  # must print nothing
    /opt/ros/jazzy/lib/tf2_ros/static_transform_publisher --x 1.0 --y 2.0 --yaw 1.5707963 \
        --frame-id map --child-frame-id base_footprint &
    /opt/ros/jazzy/lib/tf2_ros/static_transform_publisher --x 0.1 --frame-id base_footprint \
        --child-frame-id arm_base_link &
    ros2 run thorp_world world_node.py --ros-args -p world_file:=/tmp/world.yaml &
    ros2 run thorp_world check_world_node.py

Static transforms published on a domain a simulation uses stay in its nodes' buffers: never run this beside one.
"""
import json
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from moveit_msgs.msg import CollisionObject
from std_srvs.srv import Trigger
from thorp_msgs.msg import WorldState
from thorp_msgs.srv import ObserveWorld, QueryWorld, UpdateWorld
from visualization_msgs.msg import MarkerArray

rclpy.init()
node = Node('world_check')
failures = []


def check(what, ok):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        failures.append(what)


def call(srv_type, name, request):
    client = node.create_client(srv_type, name)
    assert client.wait_for_service(timeout_sec=10.0), name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    return future.result()


def detected(name, x, y, z, size, color):
    obj = CollisionObject(id=name)
    obj.header.frame_id = 'base_footprint'
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = x, y, z
    obj.pose.orientation.w = 1.0
    obj.type.db = json.dumps({'size': size, 'color': color})
    return obj


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# the robot is at (1, 2) facing +y: base_footprint (x, y) is map (1 - y, 2 + x)
request = ObserveWorld.Request()
request.surface = detected('table', 0.45, 0.0, 0.475, [0.55, 0.55, 0.02], 'white')
request.objects = [detected('cube 1', 0.3, 0.05, 0.49, [0.025] * 3, 'white'),
                   detected('star 1', 0.55, -0.1, 0.48, [0.04, 0.04, 0.01], 'red')]
response = call(ObserveWorld, 'world/observe', request)
check(f'observe: {response.surface_id} {list(response.object_ids)}',
      response.success and response.surface_id == 'table 1' and list(response.object_ids) == ['cube 1', 'star 1'])

response = call(QueryWorld, 'world/query', QueryWorld.Request(ids=['table 1', 'cube 1', 'star 1']))
by_id = {e.id: (e, r) for e, r in zip(response.entities, response.reachable)}
table, cube, star = by_id['table 1'][0], by_id['cube 1'][0], by_id['star 1'][0]
check(f'table in map at ({table.pose.pose.position.x:.3f}, {table.pose.pose.position.y:.3f}) '
      f'yaw {yaw(table.pose.pose.orientation):.3f}',
      abs(table.pose.pose.position.x - 1.0) < 1e-3 and abs(table.pose.pose.position.y - 2.45) < 1e-3 and
      abs(yaw(table.pose.pose.orientation) - math.pi / 2) < 1e-3)
check(f'cube in map at ({cube.pose.pose.position.x:.3f}, {cube.pose.pose.position.y:.3f}), on {cube.support}',
      abs(cube.pose.pose.position.x - 0.95) < 1e-3 and abs(cube.pose.pose.position.y - 2.3) < 1e-3 and
      cube.support == 'table 1')
check(f'reachable: cube {by_id["cube 1"][1]}, star {by_id["star 1"][1]} (0.21 and 0.46 m from the arm)',
      by_id['cube 1'][1] and not by_id['star 1'][1])
check(f'not docked yet: {response.docked_at!r}', response.docked_at == '')

check('dock at table 1', call(UpdateWorld, 'world/update',
                              UpdateWorld.Request(effect=UpdateWorld.Request.DOCKED, table_id='table 1')).success)
check('pick cube 1', call(UpdateWorld, 'world/update',
                          UpdateWorld.Request(effect=UpdateWorld.Request.PICKED, object_id='cube 1')).success)
bad = call(UpdateWorld, 'world/update', UpdateWorld.Request(effect=UpdateWorld.Request.PICKED, object_id='cube 9'))
check(f'picking an unknown object is refused: {bad.message!r}', not bad.success and 'cube 9' in bad.message)

response = call(QueryWorld, 'world/query', QueryWorld.Request(support='gripper'))
check(f'holding: {[e.id for e in response.entities]} ({response.entities[0].provenance}), '
      f'docked at {response.docked_at!r}',
      [e.id for e in response.entities] == ['cube 1'] and response.entities[0].provenance == 'believed' and
      not response.entities[0].has_pose and response.docked_at == 'table 1')

# the state and markers are latched: a late subscriber gets them
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
received = {}
node.create_subscription(WorldState, 'world/state', lambda m: received.setdefault('state', m), latched)
node.create_subscription(MarkerArray, 'world/markers', lambda m: received.setdefault('markers', m), latched)
for _ in range(50):
    rclpy.spin_once(node, timeout_sec=0.1)
    if len(received) == 2:
        break
state = received.get('state')
check(f'latched state: {len(state.entities) if state else None} entities, '
      f'docked at {state.docked_at if state else None!r}',
      state is not None and len(state.entities) == 3 and state.docked_at == 'table 1')
markers = received.get('markers')
check(f'markers: {len(markers.markers) if markers else None} (delete all, then box and label for table and star)',
      markers is not None and len(markers.markers) == 5)

check('save', call(Trigger, 'world/save', Trigger.Request()).success)
check('load', call(Trigger, 'world/load', Trigger.Request()).success)
response = call(QueryWorld, 'world/query', QueryWorld.Request())
check(f'after save and load: {sorted(e.id for e in response.entities)}',
      sorted(e.id for e in response.entities) == ['cube 1', 'star 1', 'table 1'])

node.destroy_node()
rclpy.shutdown()
sys.exit(1 if failures else 0)
