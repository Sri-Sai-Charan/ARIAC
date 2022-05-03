#include <vector>
#include <ros/ros.h>
#include "main_handler.h"
#include "order_handler.h"
#include "bins_handler.h"
#include "agv.h"
#include "kitting_handler.h"
#include "kitting_robot.h"
#include "conveyor.h"
#include "gantry_robot.h"
#include "assembly_handler.h"
int main(int argc, char ** argv){
    
    ros::init(argc, argv, "rwa4_node");    

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    BinsHandler bins_handler(nh);
    Conveyor conveyor(nh);

    // wait till bins handler has started
    bool is_bins_handler_initialized = false;
    nh.getParam("/is_bins_handler_initialized", is_bins_handler_initialized);
    while (!is_bins_handler_initialized)
    {
        nh.getParam("/is_bins_handler_initialized", is_bins_handler_initialized);
    }    

    AGV agv_1(nh, 1);
    AGV agv_2(nh, 2);
    AGV agv_3(nh, 3);
    AGV agv_4(nh, 4); 

    ros::Duration(3).sleep();
    nh.setParam("/is_agv_handler_initialized", true); // updating param server

    
    MainHandler main_handler(nh);  

    ros::Duration(10).sleep();

    OrderHandler order_handler(nh);
    

    // wait till order handler has started
    bool is_order_initialized = false;
    nh.getParam("/is_order_initialized", is_order_initialized);
    while (!is_order_initialized)
    {
        nh.getParam("/is_order_initialized", is_order_initialized);
    }  

    ros::Duration(5).sleep();
    ROS_INFO("Started kitting handler");   

    GantryRobot gantry_robot(nh);

    KittingRobot kitting_robot(nh);
    KittingHandler kitting_handler(nh);

    AssemblyHandler assembly_handler(nh);

    

    ros::waitForShutdown();

}

