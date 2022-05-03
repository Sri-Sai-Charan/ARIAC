#include <vector>
#include <ros/ros.h>
#include <string>
#include "std_msgs/String.h"
#include "agv.h"

int main(int argc, char ** argv){

    ros::init(argc, argv, "agv_handler_node");

    ros::NodeHandle nh;       

    AGV agv_1(nh, 1);
    AGV agv_2(nh, 2);
    AGV agv_3(nh, 3);
    AGV agv_4(nh, 4); 

    ros::spin(); 
    return 0;

}