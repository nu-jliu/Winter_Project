#!/usr/bin/env python
import rospy
import math

import sys
import moveit_commander
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory

from picker_interfaces.srv import MoveSawyer, MoveSawyerRequest, MoveSawyerResponse


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        return all_close(
            moveit_commander.conversions.pose_to_list(goal),
            moveit_commander.conversions.pose_to_list(actual),
            tolerance,
        )

    return True


class SawyerMoverNode:
    def __init__(self):
        rospy.init_node("sawyer_mover")
        rospy.loginfo("Starting SawyerMoverNode as sawyer_mover.")

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "right_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.pub_display_traj = rospy.Publisher(
            "move_group/display_planned_path", DisplayTrajectory, queue_size=20
        )

        self.srv_move_sawyer = rospy.Service(
            "move_sawyer", MoveSawyer, self.srv_move_sawyer_callback
        )

    def srv_move_sawyer_callback(self, request: MoveSawyerRequest):
        rospy.loginfo("Received request")

        move_goal = Pose()
        move_goal.position.x = request.goal.position.x
        move_goal.position.y = request.goal.position.y
        move_goal.position.z = request.goal.position.z

        q = quaternion_from_euler(math.pi, 0.0, 0.0)

        move_goal.orientation.x = q[0]
        move_goal.orientation.y = q[1]
        move_goal.orientation.z = q[2]
        move_goal.orientation.w = q[3]

        self.move_group.set_pose_target(move_goal)
        plan = self.move_group.go(wait=True)
        self.move_group.execute(plan)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        response = MoveSawyerResponse()
        response.result = True
        return response


if __name__ == "__main__":
    sawyer_mover = SawyerMoverNode()
    rospy.spin()
