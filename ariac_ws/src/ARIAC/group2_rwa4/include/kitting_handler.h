// ###################################################################
// ################### Importing Necessary Libraries #################
// ###################################################################

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

// standard library
#include <string>
#include <vector>
#include <array>
#include <cstdarg>


// group2 msg imports
#include <group2_rwa4/order_kitting_shipment_details.h>
#include <group2_rwa4/list_all_parts.h>
#include <group2_rwa4/check_exists.h>
#include "group2_rwa4/KittingLocation.h"
#include <group2_rwa4/kitting_part_details.h>
#include <group2_rwa4/check_agv_faulty_parts.h>
#include <nist_gear/AGVToAssemblyStation.h>
#include <group2_rwa4/order_completion_status.h>
#include <group2_rwa4/Task.h>
#include <std_srvs/Trigger.h>

// ####################################################################

class KittingHandler {

    public:

    /**
     * @brief Construct a new Kitting Handler object
     * 
     * @param node 
     */
    KittingHandler(ros::NodeHandle& node);

    /**
     * @brief Initalize Function
     * 
     */
    void init();

    /**
     * @brief function o fetch order details
     * 
     * @return true 
     * @return false 
     */
    bool process_order_kitting();

    /**
     * @brief Set the kitting poses object
     * 
     * @return true 
     * @return false 
     */
    bool perform_order_kitting();

    /**
     * @brief Get the part locations object
     * 
     * @return true 
     * @return false 
     */
    bool get_part_locations();
    

    private:
    ros::NodeHandle nh;

    // pubs and subs
    ros::Subscriber kitting_task_sub;

    // service responses
    group2_rwa4::order_kitting_shipment_details::Response kitting_shipment_details;
    group2_rwa4::check_exists::Response bins0_parts;
    group2_rwa4::check_exists::Response bins1_parts;


    std::vector<geometry_msgs::Pose> part_pickup_poses;
    std::vector<geometry_msgs::Pose> part_target_poses;
    std::vector<std::string> initial_preset_location;
    std::vector<std::string> final_preset_location;

    //Service clients
    ros::ServiceClient order_kitting_shipment_client;     
    ros::ServiceClient sensor_bins0_client;
    ros::ServiceClient sensor_bins1_client;
    ros::ServiceClient kitting_robot_pick_place_client;
    ros::ServiceClient order_completion_status_client;

    ros::ServiceClient task_completed_srv_client;
    ros::ServiceClient dispose_faulty_srv_client;
    ros::ServiceClient gantry_robot_pick_place_client;

    /**
     * @brief Get the target pose of a part object
     * 
     * @param target_pose 
     * @param target_frame 
     * @param child_frame_id 
     * @return geometry_msgs::Pose 
     */
    geometry_msgs::Pose get_part_target_pose(const geometry_msgs::Pose& target_pose, std::string target_frame, std::string child_frame_id); 

    void submit_agv(std::string agv_id, std::string assembly_station, std::string shipment_type); 

    void kitting_task_sub_callback(group2_rwa4::Task kitting_task);   

    void perform_part_kitting(group2_rwa4::Task kitting_task);

};
