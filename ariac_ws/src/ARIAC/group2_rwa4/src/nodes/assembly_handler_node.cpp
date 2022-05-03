#include "assembly_handler.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"assembly_handler_node");
    
    ros::NodeHandle nh;

    AssemblyHandler as1(nh);


    ros::spin();

}