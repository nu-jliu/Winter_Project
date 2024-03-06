#!/usr/bin/env python
import rospy
import math

from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class DriveBase:
    def __init__(self):
        rospy.init_node("drive_base")
        rospy.loginfo("Starting DriveBase as drive_base.")

        self.nav_goal: PoseStamped = None

        self.sub_iintial_pose = rospy.Subscriber(
            "/initialpose",
            PoseWithCovarianceStamped,
            self.sub_initial_pose_callback,
            queue_size=10,
        )

        self.srv_move_to_target = rospy.Service(
            "move_to_target",
            Trigger,
            self.srv_move_to_target_callback,
        )

        self.pub_base_goal = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )

    def sub_initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        rospy.loginfo("Received message")

        x_goal = msg.pose.pose.position.x
        y_goal = msg.pose.pose.position.y

        quat_x = msg.pose.pose.orientation.x
        quat_y = msg.pose.pose.orientation.y
        quat_z = msg.pose.pose.orientation.z
        quat_w = msg.pose.pose.orientation.w

        quat = (quat_x, quat_y, quat_z, quat_w)

        theta_goal = euler_from_quaternion(quat)[2]

        x_goal = x_goal - 1.5 * math.cos(theta_goal)
        y_goal = y_goal - 1.5 * math.sin(theta_goal)

        self.nav_goal = PoseStamped()
        self.nav_goal.header.stamp = rospy.Time.now()
        self.nav_goal.header.frame_id = "odom"

        self.nav_goal.pose.position.x = x_goal
        self.nav_goal.pose.position.y = y_goal
        self.nav_goal.pose.position.z = 0.0

        self.nav_goal.pose.orientation = msg.pose.pose.orientation

        # self.pub_base_goal.publish(self.nav_goal)

    def srv_move_to_target_callback(self, request: TriggerRequest):
        response = TriggerResponse()

        if self.nav_goal is not None:
            self.nav_goal.header.stamp = rospy.Time.now()
            self.pub_base_goal.publish(self.nav_goal)

            self.nav_goal = None
            response.success = True
        else:
            response.success = False
            response.message = "Nav goal not set"
        return response


if __name__ == "__main__":
    drive_base_node = DriveBase()
    rospy.spin()
