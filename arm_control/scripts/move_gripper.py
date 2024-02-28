#!/usr/bin/env python
import rospy

import intera_interface

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class SawyerGripperNode:
    def __init__(self):
        rospy.init_node("gripper_control")
        rospy.loginfo("Starting SawyerGripperNode as gripper_control.")

        self.robot_param = intera_interface.RobotParams()
        self.limbs = self.robot_param.get_limb_names()
        rospy.loginfo(f"Limbs: {self.limbs}")

        self.gripper = intera_interface.Gripper("right_gripper")
        rospy.loginfo(f"Gripper pos: {self.gripper.get_position()}")

        # self.gripper.close()
        # self.gripper.open()
        # self.gripper.set_holding_force(3.0)
        rospy.loginfo(f"Holding force: {self.gripper.get_force()}")

        self.srv_close_gripper = rospy.Service(
            "gripper/close", Trigger, self.srv_close_callback
        )

        self.srv_open_gripper = rospy.Service(
            "gripper/open", Trigger, self.srv_open_callback
        )

    def srv_close_callback(self, request: TriggerRequest):
        # self.gripper.close()
        self.gripper.set_position(0.0)
        rospy.loginfo("Gripper Closed")

        response = TriggerResponse()
        response.success = True
        return response

    def srv_open_callback(self, request: TriggerRequest):
        # self.gripper.open()
        self.gripper.set_position(2.0)
        rospy.loginfo("Gripper Opened")

        response = TriggerResponse()
        response.success = True
        return response


if __name__ == "__main__":
    gripper_control = SawyerGripperNode()
    rospy.spin()
