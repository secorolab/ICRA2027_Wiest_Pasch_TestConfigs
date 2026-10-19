#!/usr/bin/env python3
"""Exercise planner_server/is_path_valid for targeted planning cases.

The upstream planning tests call this service through PlannerTester. This helper keeps
that request/response boundary while allowing the scenario to assert one compact result.
"""

import argparse
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import Path
from std_msgs.msg import Bool


def pose(x, y):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    return msg


def make_path():
    path = Path()
    path.header.frame_id = 'map'
    path.poses = [pose(-2.0, -0.5), pose(-1.0, 0.5), pose(0.0, 2.0)]
    return path


def make_invalid_path():
    path = Path()
    path.header.frame_id = 'map'
    path.poses = [pose(10.0, 10.0), pose(11.0, 11.0), pose(12.0, 12.0),
                  pose(13.0, 13.0), pose(14.0, 14.0)]
    return path


def make_request(case):
    request = IsPathValid.Request()
    request.path = make_path()
    request.max_cost = 253

    if case == 'empty-path':
        request.path = Path()
    elif case in ('invalid-path', 'unknown-cells', 'max-cost-zero',
                  'invalid-pose-indices', 'stop-first-collision',
                  'stop-all-collisions', 'large-footprint'):
        request.path = make_invalid_path()
    elif case == 'unknown-cells':
        request.consider_unknown_as_obstacle = True
    elif case == 'max-cost-zero':
        request.max_cost = 0
    elif case == 'custom-footprint':
        request.footprint = '[[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]'
    elif case == 'invalid-footprint':
        request.footprint = 'invalid_footprint_string'
    elif case == 'nonexistent-layer':
        request.layer_name = 'non_existent_layer'
    elif case == 'empty-layer':
        request.layer_name = ''
    elif case == 'large-footprint':
        request.footprint = '[[1.0, 1.0], [1.0, -1.0], [-1.0, -1.0], [-1.0, 1.0]]'
    elif case == 'max-lookahead':
        request.max_lookahead_distance = 2.0
    elif case == 'stop-all-collisions':
        request.stop_at_first_collision = False
    return request


def expected(case, response):
    if case == 'empty-path':
        return not response.success and not response.is_valid
    if case in ('invalid-path', 'unknown-cells', 'max-cost-zero'):
        return response.success and not response.is_valid
    if case == 'invalid-pose-indices':
        return response.success and not response.is_valid and bool(response.invalid_pose_indices)
    if case == 'stop-first-collision':
        return response.success and not response.is_valid and len(response.invalid_pose_indices) == 1
    if case == 'stop-all-collisions':
        return response.success and not response.is_valid and len(response.invalid_pose_indices) > 1
    if case == 'invalid-footprint':
        return not response.success
    if case == 'nonexistent-layer':
        return not response.success and not response.is_valid
    if case in ('custom-footprint', 'empty-layer', 'valid-path'):
        return response.success and response.is_valid
    if case == 'large-footprint':
        return response.success and not response.is_valid and bool(response.invalid_pose_indices)
    if case == 'max-lookahead':
        return response.success and response.is_valid
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', default='none')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('planner_path_valid_helper')
    result_pub = node.create_publisher(Bool, '/planner_test/result', 10)

    passed = args.case == 'none'
    if args.case != 'none':
        client = node.create_client(IsPathValid, '/is_path_valid')
        deadline = time.monotonic() + 120.0
        while not client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        if client.service_is_ready():
            future = client.call_async(make_request(args.case))
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if future.done() and future.exception() is None:
                passed = expected(args.case, future.result())

    result = Bool()
    result.data = passed
    # Keep the verdict latched in practice for the scenario's check_data subscriber.
    end = time.monotonic() + 30.0
    while rclpy.ok() and time.monotonic() < end:
        result_pub.publish(result)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
