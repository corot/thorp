"""
thorp_world's tree nodes, run through bt_server against the world in test/world.yaml, reloaded before each test.
They need bt_server, thorp_world on that file, and transforms putting the robot docked at its table; on a ROS domain
nothing else uses, as the static transforms would reach any simulation there:

    export ROS_DOMAIN_ID=61 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ros2 run tf2_ros static_transform_publisher --x 0.03 --frame-id map --child-frame-id base_footprint &
    ros2 run tf2_ros static_transform_publisher --x 0.1 --frame-id base_footprint --child-frame-id arm_base_link &
    ros2 run thorp_world world_node.py --ros-args -p world_file:=$PWD/test/world.yaml &
    ros2 run thorp_bt_cpp bt_server_node --ros-args \
        -p bt_dir:=$(ros2 pkg prefix thorp_bt_cpp)/share/thorp_bt_cpp/bt &  # the installed trees: those ported
    python3 -m pytest test/test_world_nodes.py -v
"""

import os

import pytest
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import ParameterType
from rclpy.parameter_client import AsyncParameterClient
from std_srvs.srv import Trigger

from thorp_msgs.msg import ThorpError

from conftest import status_name

WORLD = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'world.yaml')


@pytest.fixture(scope='module', autouse=True)
def on_the_test_world(runner):
    """Skip the lot beside a robot, as on a test session of capabilities: the world isn't test/world.yaml there."""
    parameters = AsyncParameterClient(runner.node, '/world')
    if not parameters.wait_for_services(timeout_sec=5.0):
        pytest.fail('thorp_world is not running')
    future = parameters.get_parameters(['world_file'])
    assert runner.wait(future, 5.0), "reading thorp_world's world_file timed out"
    value = future.result().values[0]
    world_file = value.string_value if value.type == ParameterType.PARAMETER_STRING else ''
    if not world_file or os.path.realpath(os.path.expanduser(world_file)) != os.path.realpath(WORLD):
        pytest.skip(f"thorp_world runs on '{world_file}', not on test/world.yaml beside the static transforms the "
                    "docstring tells")


@pytest.fixture(autouse=True)
def fresh_world(runner):
    client = runner.node.create_client(Trigger, 'world/load')
    assert client.wait_for_service(timeout_sec=5.0), 'thorp_world is not running'
    future = client.call_async(Trigger.Request())
    assert runner.wait(future, 5.0), 'world/load timed out'
    assert future.result().success, future.result().message


def run(runner, tree, **inputs):
    state, out, result = runner.run(tree, inputs=inputs, output_keys=['table_id', 'table', 'table_pose', 'error'])
    assert state == GoalStatus.STATUS_SUCCEEDED, status_name(state)
    return result.success, out


def test_a_table_comes_as_a_box_of_its_size_on_the_map(runner):
    succeeded, out = run(runner, 'test_world_table', table_id='table 1')
    assert succeeded
    table, pose = out['table'], out['table_pose']
    assert table['name'] == 'table 1' and table['color'] == 'white'
    assert (table['depth'], table['width'], table['height']) == pytest.approx((0.55, 0.55, 0.02))
    assert pose['frame'] == 'map' and pose['x'] == pytest.approx(0.45)


def test_an_unknown_table_fails_with_table_not_found(runner):
    succeeded, out = run(runner, 'test_world_table', table_id='table 9')
    assert not succeeded and out['error'] == ThorpError.TABLE_NOT_FOUND


def test_the_current_table_is_the_one_the_robot_is_at(runner):
    succeeded, out = run(runner, 'test_world_current')
    assert succeeded and out['table_id'] == 'table 1'


@pytest.mark.parametrize('object_id, error', [
    ('cube 9', ThorpError.OBJECT_NOT_FOUND),
    ('cube 2', ThorpError.OBJECT_NOT_ON_TABLE),  # in the gripper
    ('pentagon 1', ThorpError.NOT_AT_TABLE),  # on table 2
    ('star 1', ThorpError.OUT_OF_REACH),  # 0.56 m from the arm
])
def test_unpickable_objects_say_why(runner, object_id, error):
    succeeded, out = run(runner, 'test_world_pickable', object_id=object_id)
    assert not succeeded and out['error'] == error


def test_a_reachable_object_on_the_docked_table_is_pickable(runner):
    succeeded, out = run(runner, 'test_world_pickable', object_id='cube 1')
    assert succeeded and out['table_id'] == 'table 1'


def test_an_effect_changes_what_the_world_says(runner):
    succeeded, _ = run(runner, 'test_world_update', effect='picked', object_id='cube 1')
    assert succeeded
    succeeded, out = run(runner, 'test_world_pickable', object_id='cube 1')
    assert not succeeded and out['error'] == ThorpError.OBJECT_NOT_ON_TABLE


def test_a_table_is_visited_once_inspected(runner):
    succeeded, _ = run(runner, 'test_world_visited', table_id='table 1')
    assert not succeeded, 'seen, but not looked over'
    succeeded, _ = run(runner, 'test_world_update', effect='inspected', table_id='table 1')
    assert succeeded
    succeeded, _ = run(runner, 'test_world_visited', table_id='table 1')
    assert succeeded


def test_an_effect_on_an_unknown_object_fails(runner):
    succeeded, _ = run(runner, 'test_world_update', effect='picked', object_id='cube 9')
    assert not succeeded


def test_an_unknown_effect_aborts_the_goal(runner):
    state, out, _ = runner.run('test_world_update', inputs={'effect': 'eaten', 'object_id': 'cube 1'})
    assert state == GoalStatus.STATUS_ABORTED and "unknown effect 'eaten'" in out.get('error', ''), out
