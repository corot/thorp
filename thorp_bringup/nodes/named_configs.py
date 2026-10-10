#!/usr/bin/env python3
"""
Named configurations: sets of other nodes' parameters, to use while doing something special, as docking at a table.
Each one is a YAML file named after it, mapping absolute node names to nested parameter trees. One is in use at a time;
using another, or the default one with an empty name, restores the parameters it doesn't set to their values before any
named configuration was in use.
"""

import os

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from thorp_msgs.srv import SetNamedConfig

TIMEOUT = 2.0  # seconds to wait for another node's parameter services


def flatten(tree, prefix=''):
    """ A nested parameter tree as dotted parameter names """
    params = {}
    for key, value in tree.items():
        name = f'{prefix}.{key}' if prefix else key
        if isinstance(value, dict):
            params.update(flatten(value, name))
        else:
            params[name] = value
    return params


def as_parameter_value(name, value, default):
    """ A YAML value as a ParameterValue of the parameter's type, so 1 can set a double """
    param_type = Parameter.Type(default.type)
    if param_type == Parameter.Type.DOUBLE and isinstance(value, int):
        value = float(value)
    elif param_type == Parameter.Type.DOUBLE_ARRAY and isinstance(value, list):
        value = [float(v) for v in value]
    try:
        return Parameter(name, param_type, value).get_parameter_value()
    except (TypeError, ValueError):
        raise RuntimeError(f'{value} is not a valid {param_type.name.lower()} for {name}')


class NamedConfigs(Node):

    def __init__(self):
        super().__init__('named_configs')
        default_path = os.path.join(get_package_share_directory('thorp_bringup'), 'param', 'named_configs')
        path = self.declare_parameter('path', default_path).value
        self.configs = {}  # configuration name: {node name: {parameter name: YAML value}}
        for file_name in sorted(os.listdir(path)):
            if file_name.endswith('.yaml'):
                with open(os.path.join(path, file_name)) as file:
                    trees = yaml.safe_load(file) or {}
                self.configs[file_name[:-5]] = {node: flatten(tree) for node, tree in trees.items()}
        self.defaults = {}  # node name: {parameter name: ParameterValue}, read before setting it the first time
        self.active = ''
        self.clients_group = ReentrantCallbackGroup()
        self.get_clients = {}
        self.set_clients = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.active_pub = self.create_publisher(String, '~/active', latched)
        self.active_pub.publish(String(data=self.active))
        self.create_service(SetNamedConfig, '~/set', self.set_cb)
        self.get_logger().info(f'Named configurations from {path}: {", ".join(self.configs)}')

    def set_cb(self, request, response):
        if request.name and request.name not in self.configs:
            response.message = f'Unknown named configuration {request.name}'
            self.get_logger().error(response.message)
            return response
        try:
            self.use(request.name)
            response.success = True
            self.get_logger().info(f'Using named configuration {request.name}' if request.name else
                                   'Default configuration restored')
        except RuntimeError as err:
            response.message = f'Using named configuration {request.name or "default"} failed: {err}'
            if request.name:
                try:
                    self.use('')
                except RuntimeError as err:
                    response.message += f'; restoring the default one failed too: {err}'
            self.get_logger().error(response.message)
        self.active_pub.publish(String(data=self.active))
        return response

    def use(self, name):
        """ Set the given configuration's parameters, and restore the defaults for the rest """
        config = self.configs.get(name, {})
        for node, params in config.items():
            unread = [param for param in params if param not in self.defaults.get(node, {})]
            if unread:
                self.defaults.setdefault(node, {}).update(self.get_params(node, unread))
        for node, defaults in self.defaults.items():
            values = dict(defaults)
            for param, value in config.get(node, {}).items():
                values[param] = as_parameter_value(param, value, defaults[param])
            self.set_params(node, values)
        self.active = name
        if not name:
            # read again next time, in case someone else changes them meanwhile
            self.defaults.clear()

    def get_params(self, node, names):
        if node not in self.get_clients:
            self.get_clients[node] = self.create_client(GetParameters, f'{node}/get_parameters',
                                                        callback_group=self.clients_group)
        response = self.call(self.get_clients[node], GetParameters.Request(names=names))
        # no values at all if any parameter isn't declared
        if len(response.values) != len(names) or any(v.type == ParameterType.PARAMETER_NOT_SET
                                                      for v in response.values):
            raise RuntimeError(f'{node} lacks some of the parameters {", ".join(names)}')
        return dict(zip(names, response.values))

    def set_params(self, node, values):
        if node not in self.set_clients:
            self.set_clients[node] = self.create_client(SetParameters, f'{node}/set_parameters',
                                                        callback_group=self.clients_group)
        request = SetParameters.Request(parameters=[ParameterMsg(name=n, value=v) for n, v in values.items()])
        response = self.call(self.set_clients[node], request)
        for param, result in zip(values, response.results):
            if not result.successful:
                raise RuntimeError(f'{node} rejected {param}: {result.reason}')

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=TIMEOUT):
            raise RuntimeError(f'{client.srv_name} not available')
        response = client.call(request, timeout_sec=TIMEOUT)
        if response is None:
            raise RuntimeError(f"{client.srv_name} didn't answer")
        return response


def main():
    rclpy.init()
    node = NamedConfigs()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
