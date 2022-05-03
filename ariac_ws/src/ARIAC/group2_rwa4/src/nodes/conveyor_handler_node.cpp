// #include <ros/ros.h>
// #include "conveyor.h"

// int main(int argc, char ** argv)
// {   
//     ros::init(argc, argv, "conveyor_handler_node");

//     ROS_INFO("Coveyor handler node initialized");
//     ros::NodeHandle nh;
//     Conveyor conveyor_handler(nh);

//     ros::spin();

// }

#include <ros/ros.h>
#include "conveyor.h"

int main(int argc, char ** argv)
{   
    ros::init(argc, argv, "conveyor_handler_node");

    ros::NodeHandle nh;
    Conveyor conveyor(nh);

    ros::spin();

}