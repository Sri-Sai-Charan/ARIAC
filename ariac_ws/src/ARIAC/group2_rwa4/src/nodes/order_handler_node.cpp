#include "order_handler.h"

int main(int argc, char ** argv){

    ros::init(argc, argv, "order_handler_node");

    ros::NodeHandle nh;
    OrderHandler order_handler(nh);

    ros::spin(); 
    return 0;

}