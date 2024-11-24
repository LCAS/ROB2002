#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import rclpy
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler

"""
A simple waypoint navigation demo where the robot visits a sequence of waypoints placed on the map.
"""

# The waypoint route, probably read in from a file for a real application
# from either a map or drive and repeat. x, y and theta (in radians)
waypoint_route = [
    [0.0, -1.75, math.pi],
    [-4.0, -1.75, math.pi/2],
    [-4.0, 0.0, 0.0],
    [-2.0, 0.0, -math.pi/2],
    [-2.0, -1.75, 0.0],
    [0.0, -1.75, math.pi/2],
    [0.0, 0.0, 0.0],
]

# the same route but expressed as x, y and angle as partial quarterion (z, w)
waypoint_route_quat = [
    [0.0, -1.75, -1.0, 0.0],
    [-4.0, -1.75, 0.7, 0.7],
    [-4.0, 0.0, 0.0, 1.0],
    [-2.0, 0.0, -0.7, 0.7],
    [-2.0, -1.75, 0.0, 1.0],
    [0.0, -1.75, 0.7, 0.7],
    [0.0, 0.0, 0.0, 1.0],
]

def pose_from_xyquat(timestamp, x=0.0, y=0.0, pz=0.0, pw=1.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = timestamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = pz
    pose.pose.orientation.w = pw
    return pose

def pose_from_xytheta(timestamp, x=0.0, y=0.0, theta=0.0):
    # negative theta: turn clockwise
    q = quaternion_from_euler(0, 0, theta)
    return pose_from_xyquat(timestamp, x, y, q[2], q[3])


def main():

    rclpy.init()

    # publish the current waypoint location (for visualisation in rviz)
    publisher = Node("waypoint_publisher").create_publisher(PoseStamped, "/current_waypoint", qos_profile=qos.qos_profile_parameters)

    navigator = BasicNavigator()

    # Set the initial pose
    initial_pose = pose_from_xyquat(navigator.get_clock().now().to_msg())

    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Prepare a set of waypoints 
    waypoints = []
    for wp in waypoint_route:
        waypoints.append(pose_from_xytheta(navigator.get_clock().now().to_msg(), *wp))

    # # if you would rather specify the orientation as quaternion use the following line instead
    # for wp in waypoint_route_quat:
    #     waypoints.append(pose_from_xyquat(navigator.get_clock().now().to_msg(), *wp))

    # follow the specified waypoints
    navigator.followWaypoints(waypoints)

    # Some feedback on the progress
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        # publish currrent waypoint
        publisher.publish(waypoints[feedback.current_waypoint])        
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(waypoints))
            )

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Task complete!')
    elif result == TaskResult.CANCELED:
        print('Task was canceled.')
    elif result == TaskResult.FAILED:
        print('Task failed!')


if __name__ == '__main__':
    main()
