#!/usr/bin/env python
import PySimpleGUI

import rospy

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


def wait_until_service_arrive(topic):
    while True:
        try:
            rospy.wait_for_service(topic, timeout=rospy.Duration(secs=2.0))
        except rospy.ROSException:
            rospy.loginfo(
                f"Service with topic: {topic} not available, waiting again ..."
            )
            continue
        finally:
            break


if __name__ == "__main__":
    rospy.init_node("gui_app")
    rospy.loginfo("Starting gui_app.")

    layout = [
        [
            PySimpleGUI.Button("Pick"),
            PySimpleGUI.Button("Place"),
            PySimpleGUI.Button("Ready"),
        ],
        [PySimpleGUI.Button("Open Gripper"), PySimpleGUI.Button("Close Gripper")],
        [
            PySimpleGUI.Button("Look Ahead"),
            PySimpleGUI.Button("Look Left"),
            PySimpleGUI.Button("Retract"),
        ],
    ]
    window = PySimpleGUI.Window(title="Commander", layout=layout)

    cli_pick = rospy.ServiceProxy("pick", Trigger)
    cli_place = rospy.ServiceProxy("place", Trigger)
    cli_ready = rospy.ServiceProxy("go_ready", Trigger)
    cli_close_gripper = rospy.ServiceProxy("gripper/close", Trigger)
    cli_open_gripper = rospy.ServiceProxy("gripper/open", Trigger)
    cli_look_forward = rospy.ServiceProxy("look_forward", Trigger)
    cli_look_left = rospy.ServiceProxy("look_left", Trigger)
    cli_px100_sleep = rospy.ServiceProxy("sleep_px100", Trigger)

    wait_until_service_arrive("pick")
    wait_until_service_arrive("place")
    wait_until_service_arrive("go_ready")
    wait_until_service_arrive("gripper/close")
    wait_until_service_arrive("gripper/open")
    wait_until_service_arrive("look_forward")
    wait_until_service_arrive("look_left")
    wait_until_service_arrive("sleep_px100")

    rate = rospy.Rate(hz=100.0)

    while not rospy.is_shutdown():

        event, values = window.read()

        if event == PySimpleGUI.WIN_CLOSED:
            break

        elif event == "Pick":
            request = TriggerRequest()
            rospy.loginfo("Sending request pick")

            try:
                response: TriggerResponse = cli_pick.call(request)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            finally:
                rospy.loginfo(f"Received response: {response}")

        elif event == "Place":
            request = TriggerRequest()
            rospy.loginfo("Sending request place")

            try:
                response: TriggerResponse = cli_place.call(request)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            finally:
                rospy.loginfo(f"Received response: {response}")

        elif event == "Ready":
            request = TriggerRequest()
            rospy.loginfo("Sending request go to ready")

            try:
                response: TriggerResponse = cli_ready.call(request)
            except:
                rospy.logerr(f"Service call failed: {e}")
            finally:
                rospy.loginfo(f"Received response: {response}")

        elif event == "Open Gripper":
            request = TriggerRequest()
            rospy.loginfo("Sending request open gripper")

            try:
                response: TriggerResponse = cli_open_gripper.call(request)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            finally:
                rospy.loginfo(f"Received response: {response}")

        elif event == "Close Gripper":
            request = TriggerRequest()
            rospy.loginfo("Sending request close gripper")

            try:
                response: TriggerResponse = cli_close_gripper.call(request)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            finally:
                rospy.loginfo(f"Received response: {response}")

        elif event == "Look Ahead":
            request = TriggerRequest()
            rospy.loginfo("Sending request look head")

            try:
                response: TriggerResponse = cli_look_forward.call(request)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            finally:
                rospy.loginfo(f"Received response: {response}")

        elif event == "Look Left":
            request = TriggerRequest()
            rospy.loginfo("Sending request look left")

            try:
                response: TriggerResponse = cli_look_left.call(request)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            finally:
                rospy.loginfo(f"Received response: {response}")

        elif event == "Retract":
            request = TriggerRequest()
            rospy.loginfo("Sending request retract")

            try:
                response: TriggerResponse = cli_px100_sleep.call(request)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            finally:
                rospy.loginfo(f"Received response: {response}")

        rate.sleep()
