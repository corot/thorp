#!/usr/bin/env python3

import sys
import rclpy

from thorp_msgs.srv import CannonCommand


def cannon_command(node, cmd, arg):
    srv = node.create_client(CannonCommand, 'cannon_command')
    srv.wait_for_service()
    # arg is the angle for TILT and the number of shots for FIRE; shots is unsigned
    request = CannonCommand.Request(action=cmd)
    if cmd == CannonCommand.Request.TILT:
        request.angle = float(arg)
    elif cmd == CannonCommand.Request.FIRE:
        request.shots = arg
    future = srv.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print(future.result())
    else:
        print("Service call failed: %s" % future.exception())


def usage():
    return "%s <cmd> <arg>" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 3:
        cmd = int(sys.argv[1])
        arg = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)

    rclpy.init()
    node = rclpy.create_node('cannon_command')
    print("Requesting %d %d" % (cmd, arg))
    cannon_command(node, cmd, arg)
    node.destroy_node()
    rclpy.shutdown()
