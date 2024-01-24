#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        
    
        ros::spinOnce();
        loop_rate.sleep();
    }
}