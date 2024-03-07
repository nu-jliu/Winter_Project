#!/usr/bin/env python
import rospy

from sensor_msgs.msg import JointState


class JointStateNode:
    def __init__(self):
        rospy.init_node("joint_state_republisher")
        rospy.loginfo("Starting JointStateNode.")

        self.sawyer_joint_states: JointState = None
        self.ridgeback_joint_states: JointState = None

        self.sub_sawyer_joint_states = rospy.Subscriber(
            "/robot/joint_states",
            JointState,
            self.sub_sawyer_joint_states_callback,
        )
        self.sub_ridgeback_joint_states = rospy.Subscriber(
            "/ridgeback/joint_states",
            JointState,
            self.sub_ridgeback_joint_states_callback,
        )

        self.pub_joint_states = rospy.Publisher(
            "joint_states",
            JointState,
            queue_size=10,
        )

        self.timer = rospy.Timer(rospy.Duration(1.0 / 100.0), self.timer_callback)

    def timer_callback(self, event):
        rospy.loginfo("Timer event")

        if (
            self.sawyer_joint_states is not None
            and self.ridgeback_joint_states is not None
        ):
            joint_states = JointState()

            joint_states.header.stamp = rospy.Time.now()
            joint_states.header.frame_id = ""

            joint_states.name = (
                self.ridgeback_joint_states.name + self.sawyer_joint_states.name
            )
            joint_states.position = (
                self.ridgeback_joint_states.position + self.sawyer_joint_states.position
            )
            joint_states.velocity = (
                self.ridgeback_joint_states.velocity + self.sawyer_joint_states.velocity
            )
            joint_states.effort = (
                self.ridgeback_joint_states.effort + self.sawyer_joint_states.effort
            )

            self.pub_joint_states.publish(joint_states)

    def sub_sawyer_joint_states_callback(self, msg: JointState):
        self.sawyer_joint_states = msg
        self.sawyer_joint_states.header.stamp = rospy.Time.now()

    def sub_ridgeback_joint_states_callback(self, msg: JointState):
        self.ridgeback_joint_states = msg
        self.ridgeback_joint_states.header.stamp = rospy.Time.now()


if __name__ == "__main__":
    joint_state_republisher = JointStateNode()
    rospy.spin()
