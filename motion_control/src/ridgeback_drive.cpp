#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ridgeback_drive");
    ros::NodeHandle nh;
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.1;

        ROS_INFO("Sending cmd_vel %f", msg.linear.x);
        pub_cmd_vel.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    nh.shutdown();
    return 0;
}
