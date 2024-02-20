#!/usr/bin/env python
import math

import sys
import rospy
import moveit_commander

import tf

from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, PoseStamped


if __name__ == "__main__":
    rospy.init_node("mover_ik", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    goal_pose = rospy.get_param("~goal_pose", {"x": 0.0, "y": 0.0, "z": 0.0})
    # goal_y = rospy.get_param("~goal_y", 0.0)
    # goal_z = rospy.get_param("~goal_z", 0.0)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_pub = rospy.Publisher(
        "move_group/display_planned_path", DisplayTrajectory, queue_size=20
    )

    box_base = PoseStamped()
    box_base.header.frame_id = "base"

    box_base.pose.position.x = 0.0
    box_base.pose.position.y = 0.0
    box_base.pose.position.z = -0.5

    box_base.pose.orientation.x = 0.0
    box_base.pose.orientation.y = 0.0
    box_base.pose.orientation.z = 0.0
    box_base.pose.orientation.w = 1.0

    # scene.add_box("base_box", box_base, size=(2.0, 1.0, 1.0))

    pose_goal = Pose()
    pose_goal.position.x = goal_pose["x"]
    pose_goal.position.y = goal_pose["y"]
    pose_goal.position.z = goal_pose["z"]
    q = tf.transformations.quaternion_from_euler(
        math.pi,
        0.0,
        0.0,
    )
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.execute(plan, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
