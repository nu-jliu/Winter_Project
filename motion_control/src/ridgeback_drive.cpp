#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"

// void sub_clicked_point_callback(const geometry_msgs::PointStamped &msg)
// {
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ridgeback_drive");
    ros::NodeHandle nh;
    ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::Subscriber sub_clicked_point;
    // sub_clicked_point = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &sub_clicked_point_callback);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y = 1.0;
        msg.linear.z = 0.0;

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

        ROS_INFO("Sending cmd_vel %f", msg.linear.x);
        pub_cmd_vel.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    nh.shutdown();
    return 0;
}
