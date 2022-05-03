#include <vector>
#include <ros/ros.h>
#include "main_handler.h"

int main(int argc, char ** argv){
    
    ros::init(argc, argv, "main_handler");    

    ros::NodeHandle nh;
    MainHandler main_handler(nh);

    ros::spin(); 
    return 0;

}