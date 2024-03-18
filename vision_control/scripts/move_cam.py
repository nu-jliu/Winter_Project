#!/usr/bin/env python
from interbotix_xs_modules.arm import InterbotixManipulatorXS

import rospy

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image

from picker_interfaces.srv import MoveCamera, MoveCameraRequest, MoveCameraResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class CameraMoverNode:
    def __init__(self):
        self.interbotix = InterbotixManipulatorXS("px100", "arm", "gripper")
        # rospy.init_node("camera_mover")
        rospy.loginfo("Starting CameraMoverNode as camera_mover.")

        self.tf_static_broadcaster = StaticTransformBroadcaster()

        # tf_base = TransformStamped()
        # tf_base.header.stamp = rospy.Time.now()
        # tf_base.header.frame_id = "mid_mount"
        # tf_base.child_frame_id = "px100/base_link"

        # tf_base.transform.translation.x = 0.140
        # tf_base.transform.translation.y = -0.3365
        # tf_base.transform.translation.z = 0.008

        # tf_base.transform.rotation.x = 0.0
        # tf_base.transform.rotation.y = 0.0
        # tf_base.transform.rotation.z = 0.0
        # tf_base.transform.rotation.w = 1.0

        tf_cam = TransformStamped()
        tf_cam.header.stamp = rospy.Time.now()
        tf_cam.header.frame_id = "px100/ee_arm_link"
        tf_cam.child_frame_id = "d435_link"

        tf_cam.transform.translation.x = 11e-3 - 5.709e-3
        tf_cam.transform.translation.y = 0.0
        tf_cam.transform.translation.z = (28.0 + 25.192 - 23.0 / 2) / 1000.0

        tf_cam.transform.rotation.x = 0.0
        tf_cam.transform.rotation.y = 0.0
        tf_cam.transform.rotation.z = 0.0
        tf_cam.transform.rotation.w = 1.0

        # self.tf_static_broadcaster.sendTransform(transform=tf_base)
        self.tf_static_broadcaster.sendTransform(transform=tf_cam)

        self.srv_move_cam = rospy.Service(
            "move_cam",
            MoveCamera,
            self.srv_move_cam_callback,
        )
        self.srv_look_forward = rospy.Service(
            "look_forward",
            Trigger,
            self.srv_look_forward_callback,
        )
        self.srv_look_left = rospy.Service(
            "look_left",
            Trigger,
            self.srv_look_left_callback,
        )
        self.srv_sleep_px100 = rospy.Service(
            "sleep_px100",
            Trigger,
            self.srv_sleep_callback,
        )

        self.sub_cam_image = rospy.Subscriber(
            "/d435/color/image_raw",
            Image,
            self.sub_image_callback,
            queue_size=10,
        )
        self.pub_image = rospy.Publisher("/robot/head_display", Image, queue_size=10)

    def srv_move_cam_callback(self, request: MoveCameraRequest):

        x = request.point.x
        y = request.point.y
        z = request.point.z

        self.interbotix.arm.set_ee_pose_components(x=x, y=y, z=z)

        response = MoveCameraResponse()
        response.success = True
        return response

    def srv_look_forward_callback(self, request: TriggerRequest):

        self.interbotix.arm.set_ee_pose_components(x=0.2, y=0.0, z=0.1)

        response = TriggerResponse()
        response.success = True
        return response

    def srv_look_left_callback(self, request: TriggerRequest):

        self.interbotix.arm.set_ee_pose_components(x=0.0, y=-0.2, z=0.1)

        response = TriggerResponse()
        response.success = True
        return response

    def srv_sleep_callback(self, request: TriggerRequest):

        self.interbotix.arm.go_to_sleep_pose()

        response = TriggerResponse()
        response.success = True
        return response

    def sub_image_callback(self, msg: Image):

        image = msg
        self.pub_image.publish(image)


if __name__ == "__main__":
    camera_mover = CameraMoverNode()
    rospy.spin()
