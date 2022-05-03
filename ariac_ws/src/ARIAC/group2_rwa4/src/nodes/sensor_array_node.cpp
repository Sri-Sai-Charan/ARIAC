#include "sensor_array.h"

// struct mystruct
// {
//     std::vector<nist_gear::Model> conveyor;
//     std::vector<nist_gear::Model> bins;
//     std::vector<nist_gear::Model> agv;
//     std::vector<nist_gear::Model> assembly_station;
// };
 




int main(int argc, char ** argv){

    ros::init(argc, argv, "sensor_array_node");

    ros::NodeHandle nh;       

    sensor_array::BinSensors bins_sensors(nh);
    
    sensor_array::AgvSensors agv_sensors(nh);
    sensor_array::ConveyorSensors conveyor_parts(nh);
  
    ros::spin();
    return 0;

}