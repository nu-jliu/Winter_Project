#!/usr/bin/env python
import rospy
import math

import sys
import moveit_commander
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped
import copy

from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState

from picker_interfaces.srv import MoveSawyer, MoveSawyerRequest, MoveSawyerResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


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

        self.goal: Point = None
        self.prev_goal: Point = None

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "right_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.pub_display_traj = rospy.Publisher(
            "move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=20,
        )

        self.srv_move_sawyer = rospy.Service(
            "move_sawyer",
            MoveSawyer,
            self.srv_move_sawyer_callback,
        )
        self.srv_pick = rospy.Service("pick", Trigger, self.srv_pick_callback)
        self.srv_place = rospy.Service("place", Trigger, self.srv_place_callback)
        self.srv_go_ready = rospy.Service(
            "go_ready",
            Trigger,
            self.srv_go_ready_callback,
        )

        self.sub_clicked_point = rospy.Subscriber(
            "/clicked_point",
            PointStamped,
            self.sub_clicked_point_callback,
            queue_size=10,
        )
        self.sub_robot_joint_states = rospy.Subscriber(
            "/robot/joint_states",
            JointState,
            self.sub_robot_joint_states_callback,
            queue_size=10,
        )

        self.pub_new_joint_state = rospy.Publisher(
            "/new/joint_states",
            JointState,
            queue_size=10,
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
        try:
            self.move_group.execute(plan)
        except Exception as e:
            rospy.logerr(f"Got exception {e}")
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        response = MoveSawyerResponse()
        response.result = True
        return response

    def srv_pick_callback(self, request: TriggerRequest):
        response = TriggerResponse()

        if self.goal is not None:
            goal_standoff = Pose()
            goal_standoff.position.x = self.goal.x + 0.05
            goal_standoff.position.y = self.goal.y
            goal_standoff.position.z = 0.3

            q = quaternion_from_euler(math.pi, 0.0, 0.0)

            goal_standoff.orientation.x = q[0]
            goal_standoff.orientation.y = q[1]
            goal_standoff.orientation.z = q[2]
            goal_standoff.orientation.w = q[3]

            self.move_group.set_pose_target(goal_standoff)
            plan = self.move_group.go(wait=True)
            try:
                self.move_group.execute(plan)
            except Exception as e:
                rospy.logerr(f"Got exception {e}")
            self.move_group.stop()
            self.move_group.clear_pose_targets()

            goal_target = Pose()
            goal_target.position.x = self.goal.x + 0.05
            goal_target.position.y = self.goal.y + 0.02
            goal_target.position.z = self.goal.z - 0.02

            q = quaternion_from_euler(math.pi, 0.0, 0.0)

            goal_target.orientation.x = q[0]
            goal_target.orientation.y = q[1]
            goal_target.orientation.z = q[2]
            goal_target.orientation.w = q[3]

            self.move_group.set_pose_target(goal_target)
            plan = self.move_group.go(wait=True)
            try:
                self.move_group.execute(plan)
            except Exception as e:
                rospy.logerr(f"Got exception {e}")
            self.move_group.stop()
            self.move_group.clear_pose_targets()

            response.success = True
            self.prev_goal = copy.deepcopy(self.goal)
            self.goal = None
        else:
            response.success = False
            response.message = "Goal not set"

        return response

    def srv_place_callback(self, request: TriggerRequest):
        response = TriggerResponse()

        if self.goal is not None and self.prev_goal is not None:
            goal_standoff = Pose()
            goal_standoff.position.x = self.prev_goal.x + 0.05
            goal_standoff.position.y = self.prev_goal.y
            goal_standoff.position.z = 0.5

            q = quaternion_from_euler(math.pi, 0.0, 0.0)

            goal_standoff.orientation.x = q[0]
            goal_standoff.orientation.y = q[1]
            goal_standoff.orientation.z = q[2]
            goal_standoff.orientation.w = q[3]

            self.move_group.set_pose_target(goal_standoff)
            plan = self.move_group.go(wait=True)
            try:
                self.move_group.execute(plan)
            except Exception as e:
                rospy.logerr(f"Got exception {e}")
            self.move_group.stop()
            self.move_group.clear_pose_targets()

            goal_drop = Pose()
            goal_drop.position.x = self.goal.x
            goal_drop.position.y = self.goal.y
            goal_drop.position.z = self.goal.z + 0.1

            goal_drop.orientation.x = math.sqrt(2.0) / 2.0
            goal_drop.orientation.y = math.sqrt(2.0) / 2.0
            goal_drop.orientation.z = 0.0
            goal_drop.orientation.w = 0.0

            self.move_group.set_pose_target(goal_drop)
            plan = self.move_group.go(wait=True)
            try:
                self.move_group.execute(plan)
            except Exception as e:
                rospy.logerr(f"Got exception {e}")
            self.move_group.stop()
            self.move_group.clear_pose_targets()

            response.success = True
        else:
            response.success = False
            response.message = "Goal not set"

        return response

    def srv_go_ready_callback(self, request: TriggerRequest):
        # 0.135173828125, -1.345302734375, -0.330572265625, 1.23278515625, -0.484779296875, 1.7931494140625, 1.6405126953125

        # joint_goal = self.move_group.get_current_joint_values()
        joint_goal = []
        joint_goal.append(0.135173828125)
        joint_goal.append(-1.345302734375)
        joint_goal.append(-0.330572265625)
        joint_goal.append(1.23278515625)
        joint_goal.append(-0.484779296875)
        joint_goal.append(1.7931494140625)
        joint_goal.append(1.6405126953125)
        rospy.loginfo(f"Current goal: {joint_goal}")

        plan = self.move_group.go(joint_goal, wait=True)
        try:
            self.move_group.execute(plan)
        except Exception as e:
            rospy.logerr(f"Got exception: {e}")
        self.move_group.stop()

        response = TriggerResponse()
        response.success = True
        return response

    def sub_clicked_point_callback(self, msg: PointStamped):
        self.goal = msg.point

    def sub_robot_joint_states_callback(self, msg: JointState):
        new_joint_states = copy.deepcopy(msg)
        new_joint_states.header.stamp = rospy.Time.now()

        self.pub_new_joint_state.publish(new_joint_states)


if __name__ == "__main__":
    sawyer_mover = SawyerMoverNode()
    rospy.spin()
