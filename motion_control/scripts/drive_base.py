#!/usr/bin/env python
import rospy
import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


class DriveBase:
    def __init__(self):
        rospy.init_node("drive_base")
        rospy.loginfo("Starting DriveBase as drive_base.")

        self.sub_iintial_pose = rospy.Subscriber(
            "/initialpose",
            PoseWithCovarianceStamped,
            self.sub_initial_pose_callback,
            queue_size=10,
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

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "odom"

        goal.pose.position.x = x_goal
        goal.pose.position.y = y_goal
        goal.pose.position.z = 0.0

        goal.pose.orientation = msg.pose.pose.orientation

        self.pub_base_goal.publish(goal)


if __name__ == "__main__":
    drive_base_node = DriveBase()
    rospy.spin()
