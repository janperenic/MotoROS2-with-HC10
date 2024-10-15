#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint

class MoveGroupClient(Node):
    def __init__(self):
        super().__init__('move_group_client')

        # Create an action client for the MoveGroup action
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

        # Wait for the action server to be available
        print("waiting")
        self._action_client.wait_for_server()
        print("done")

    def move_robot(self):
        # Create a goal message
        goal_msg = MoveGroup.Goal()

        # Specify the move group options (e.g., 'arm' as the group)
        goal_msg.request.group_name = 'manipulator'
        goal_msg.request.planner_id = 'LIN'

        # Specify the target pose
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        #goal_msg.request.goal_constraints.resize(1)
        constraint = Constraints()

        # Create position and orientation constraints (optional)
        position_constraint = PositionConstraint()
        orientation_constraint = OrientationConstraint()
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.pose.position.x = 0.3
        pose.pose.position.y = 0.3
        pose.pose.position.z = 0.3
        #goal_msg.request.goal_constraints[0].position_constraints.resize(1)
        #goal_msg.request.goal_constraints[0].orientation_constraints.resize(1)

        constraint.position_constraints.append(position_constraint)
        constraint.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(constraint)

        # Set the planner ID to use Pilz 'LIN'
        #goal_msg.planning_options.planner_id = 'LIN'

        # Send the goal
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    print("test 1")
    rclpy.init(args=args)

    print("test 1")
    move_group_client = MoveGroupClient()

    print("test 1")
    move_group_client.move_robot()
    print("test 1")

    rclpy.spin(move_group_client)
    print("test 1")

    move_group_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
