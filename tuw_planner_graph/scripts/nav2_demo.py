#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped


class DemoActionClient(Node):

    def __init__(self):
        super().__init__('nav2_through_poses_action_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')

def get_goal_pose(x, y):
    g = PoseStamped()
    g.pose.position.x = x
    g.pose.position.y = y
    g.pose.position.z = 0.0
    g.pose.orientation.w = 0.707
    g.pose.orientation.x = 0.707
    g.pose.orientation.y = 0.0
    g.pose.orientation.z = 0.0
    g.header.frame_id = 'map'

    return g


def main(args=None):
    rclpy.init(args=args)

    action_client = DemoActionClient()

    msg = NavigateThroughPoses.Goal()

    msg.poses = []
    msg.poses.append(get_goal_pose(-2.695, -33.796))
    msg.poses.append(get_goal_pose(-28.302, -31.514))
    msg.poses.append(get_goal_pose(-33.223, -28.496))
    msg.poses.append(get_goal_pose(-43.971, -27.438))
    msg.poses.append(get_goal_pose(-47.944, -24.672))
    msg.poses.append(get_goal_pose(-82.373, -21.617))
    msg.poses.append(get_goal_pose(-80.088, -19.519))
    msg.poses.append(get_goal_pose(-2.103, -26.207))
    msg.poses.append(get_goal_pose(-1.165, -23.556))
    msg.poses.append(get_goal_pose(-80.618, -16.680))
    msg.poses.append(get_goal_pose(-79.390, -14.108))
    msg.poses.append(get_goal_pose(0.048, -21.079,))
    msg.poses.append(get_goal_pose(-3.078, -28.784))
    msg.poses.append(get_goal_pose(-43.538, -25.076))
    msg.poses.append(get_goal_pose(-27.543, -29.056))
    msg.poses.append(get_goal_pose(-2.049, -31.509))


    action_client._action_client.wait_for_server()

    future =  action_client._action_client.send_goal_async(msg)
    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
