#include "ros/ros.h"

#include "sensor_msgs/JointState.h"
#include "moveit_msgs/GetPositionIK.h"
#include "moveit_msgs/RobotState.h"

sensor_msgs::JointState joint_states;
moveit_msgs::RobotState robot_state;

void sub_joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_states = *msg;
    ros::Time current = ros::Time::now();

    robot_state.joint_state = joint_states;

    ROS_INFO("got joint states at time %d, %d", joint_states.header.stamp.sec, joint_states.header.stamp.nsec);
    ROS_WARN("Current time: %d, %d", current.sec, current.nsec);
}

void compute_ik(const ros::ServiceClient &client)
{
    moveit_msgs::GetPositionIK srv_position_ik;
    srv_position_ik.request.ik_request.group_name = "right_arm";
    srv_position_ik.request.ik_request.robot_state = robot_state;
    srv_position_ik.request.ik_request.avoid_collisions = true;
    srv_position_ik.request.ik_request.ik_link_name = "right_hand";

    srv_position_ik.request.ik_request.pose_stamped.header.stamp = ros::Time::now();
    srv_position_ik.request.ik_request.pose_stamped.pose.position.x = 0.5;
    srv_position_ik.request.ik_request.pose_stamped.pose.position.y = 0.0;
    srv_position_ik.request.ik_request.pose_stamped.pose.position.z = 0.4;

    srv_position_ik.request.ik_request.pose_stamped.pose.orientation.x = 0.0;
    srv_position_ik.request.ik_request.pose_stamped.pose.orientation.y = 0.0;
    srv_position_ik.request.ik_request.pose_stamped.pose.orientation.z = 0.0;
    srv_position_ik.request.ik_request.pose_stamped.pose.orientation.w = 1.0;

    srv_position_ik.request.ik_request.timeout.sec = 10.0;

    // if (client.call(srv_position_ik))
    // {
    //     ROS_INFO("Received solution");
    // }
    // else
    // {
    //     ROS_ERROR("Error, not able to call service");
    // }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_control");
    ros::NodeHandle nh;

    ros::Subscriber sub_joint_states = nh.subscribe("robot/joint_states", 1000, sub_joint_states_callback);
    ros::Publisher pub_joint_states = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

    ros::ServiceClient cli_compute_ik = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

    compute_ik(cli_compute_ik);

    ros::Rate rate(100.0);

    while (ros::ok())
    {
        pub_joint_states.publish(joint_states);

        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}