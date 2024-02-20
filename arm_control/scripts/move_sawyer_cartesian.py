import sys
import math

import rospy
import tf

import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("mover_cartesian", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "right_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_pub = rospy.Publisher(
        "move_group/display_planned_path", DisplayTrajectory, queue_size=20
    )

    q = tf.transformations.quaternion_from_euler(math.pi, 0.0, 0.0)

    waypoints = []  # List of Pose

    pose1 = Pose()
    pose1.position.x = 0.5
    pose1.position.y = 0.5
    pose1.position.z = 0.5

    pose1.orientation.x = q[0]
    pose1.orientation.y = q[1]
    pose1.orientation.z = q[2]
    pose1.orientation.w = q[3]

    pose2 = Pose()
    pose2.position.x = 0.5
    pose2.position.y = 0.0
    pose2.position.z = 0.5

    pose2.orientation.x = q[0]
    pose2.orientation.y = q[1]
    pose2.orientation.z = q[2]
    pose2.orientation.w = q[3]

    pose3 = Pose()
    pose3.position.x = 0.5
    pose3.position.y = -0.5
    pose3.position.z = 0.5

    pose3.orientation.x = q[0]
    pose3.orientation.y = q[1]
    pose3.orientation.z = q[2]
    pose3.orientation.w = q[3]

    waypoints.append(pose1)
    waypoints.append(pose2)
    waypoints.append(pose3)

    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        eef_step=0.05,
        jump_threshold=0.1,
    )
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_pub.publish(display_trajectory)
    move_group.execute(plan, wait=True)
