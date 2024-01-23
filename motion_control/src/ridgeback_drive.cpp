#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char ** argv) {
    ros::init(argc, argv, "ridgeback_drive");
    ros::NodeHandle n;
    ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("ridgeback_velocity_controller/cmd_vel", 1000);

    ros::Rate loop_rate(100);

    while (ros::ok()) {
        geometry_msgs::Twist msg;
        msg.linear.x = 1.0;
        msg.linear.y = 0.0;

        ROS_INFO("Sending cmd_vel %f", msg.linear.x);
        pub_cmd_vel.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    n.shutdown();
    return 0;
}