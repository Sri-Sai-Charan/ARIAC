#include <ros/ros.h>
#include "bins_handler.h"

int main(int argc, char ** argv)
{   
    ros::init(argc, argv, "bins_handler_node");

    ros::NodeHandle nh;
    BinsHandler bins_handler(nh);

    ros::spin();

}