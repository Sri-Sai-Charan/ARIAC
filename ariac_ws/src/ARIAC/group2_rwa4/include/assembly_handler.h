
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

// standard library
#include <string>
#include <vector>
#include <array>
#include <cstdarg>


// group2 msg imports
#include <group2_rwa4/order_assembly_shipment_details.h>
#include <group2_rwa4/list_all_parts.h>
#include <group2_rwa4/check_exists.h>
#include "group2_rwa4/KittingLocation.h"
#include <group2_rwa4/assembly_part_details.h>
#include <group2_rwa4/order_completion_status.h>
#include <group2_rwa4/agv_details_at_as.h>
#include <group2_rwa4/Task.h>
#include <group2_rwa4/assembly_task.h>
#include <group2_rwa4/kitting_part_details.h>


#include <nist_gear/Model.h>
#include <nist_gear/LogicalCameraImage.h>
// ###################################################################


/**
 * @brief Class for handling Assembly operation
 * 
 */
class AssemblyHandler {

    public:

    /**
     * @brief Construct a new Kitting Handler object
     * 
     * @param node 
     */
    AssemblyHandler(ros::NodeHandle& node);

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
    bool process_order_assembly();

    /**
     * @brief Set the kitting poses object
     * 
     * @return true 
     * @return false 
     */
    void perform_part_assembly();

    /**
     * @brief Get the part locations object
     * 
     * @return true 
     * @return false 
     */
    bool get_part_locations();

    void assembly_task_sub_callback(group2_rwa4::Task kitting_task);


    private:
    ros::NodeHandle nh;
    std::vector<nist_gear::Model> assembly_logical_cams_data[8];
    group2_rwa4::Task assembly_task_data;
        // pubs and subs
    ros::ServiceClient assembly_task_client;
    std::vector<ros::Subscriber> assembly_cam_subs;
    int task_current =0;

    struct assembly_part_location   
    {
        std::string initial_preset_location;
        geometry_msgs::Pose part_pose;
    };
    assembly_part_location location_;
    
    // service responses
    
    // group2_rwa4::order_assembly_shipment_details::Response assembly_shipment_details;


    std::vector<geometry_msgs::Pose> part_pickup_poses;
    std::vector<geometry_msgs::Pose> part_target_poses;
    std::vector<std::string> initial_preset_location;
    std::vector<std::string> final_preset_location;

    //Service clients
    ros::ServiceClient order_assembly_shipment_client;
    ros::ServiceClient gantry_robot_pick_place_client;
    ros::ServiceClient order_completion_status_client;
    ros::ServiceClient agv_details_at_as_client;
    /**
     * @brief Get the target pose of a part object
     * 
     * @param target_pose 
     * @param target_frame 
     * @param child_frame_id 
     * @return geometry_msgs::Pose 
     */
    geometry_msgs::Pose get_part_target_pose(const geometry_msgs::Pose& target_pose, std::string target_frame, std::string child_frame_id);
    void logical_camera_sub_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg,int cam_id);
    bool assembly_handler_details_at_as_service_callback(group2_rwa4::agv_details_at_as::Request &req, group2_rwa4::agv_details_at_as::Response &res);
    int find_part(assembly_part_location *current_part,std::string required_part,std::string initial_preset_location);
};