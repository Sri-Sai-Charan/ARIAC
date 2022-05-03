// ###################################################################
// ################### Importing Necessary Libraries #################
// ###################################################################

#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <nist_gear/AGVToAssemblyStation.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Model.h>
#include <group2_rwa4/check_exists.h>
#include <group2_rwa4/check_part_pose.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>


/**
 * @brief The Bins class handles the details of all spawned parts on the bins 
 * 
 */

class BinsHandler{

    private:
    ros::NodeHandle nh;
    
    // Subscribers and Services
    ros::Subscriber logicam_bin0_sub;
    ros::Subscriber logicam_bin1_sub;  
    ros::Subscriber conveyor_parts_sub;
    ros::ServiceServer check_part_service_bins0; 
    ros::ServiceServer check_part_service_bins1;
    ros::ServiceServer check_part_pose_service_bins0; 
    ros::ServiceServer check_part_pose_service_bins1;

    int bins0_parts_count; 
    std::vector<nist_gear::Model> bins0_parts;
    int bins1_parts_count; 
    std::vector<nist_gear::Model> bins1_parts;

    public:    

    /**
     * @brief Construct a new Bins Handler object
     * 
     * @param node_handler 
     */


    BinsHandler(ros::NodeHandle &node_handler);

    void init();

    /**
     * @brief Callback function for bin 1
     * 
     * @param logicam_msg 
     */

    void logicam_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);

    /**
     * @brief Callback function for bin 1
     * 
     * @param logicam_msg 
     */

    void logicam_bins1_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);

    /**
     * @brief Callback function to check requested part service for bin0
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */

    bool check_part_service_bins0_callback(group2_rwa4::check_exists::Request &req, group2_rwa4::check_exists::Response &res);

    /**
     * @brief Callback function to check requested part service for bin1
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */

    bool check_part_service_bins1_callback(group2_rwa4::check_exists::Request &req, group2_rwa4::check_exists::Response &res);

    /**
     * @brief Callback function to check requested part pose service for bin0
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */

    bool check_part_pose_service_bins0_callback(group2_rwa4::check_part_pose::Request &req, group2_rwa4::check_part_pose::Response &res);

    /**
     * @brief Callback function to check requested part pose service for bin1
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */

    bool check_part_pose_service_bins1_callback(group2_rwa4::check_part_pose::Request &req, group2_rwa4::check_part_pose::Response &res);

    /**
     * @brief Function to transform to world frame
     * 
     * @param part_pose 
     * @param logicam_frame 
     * @return geometry_msgs::Pose 
     */
    geometry_msgs::Pose transform_to_world_frame(const geometry_msgs::Pose& part_pose, std::string logicam_frame); 

    /**
     * @brief Function to constantly update conveyor part
     * 
     * @param part_pose 
     * @param logicam_frame 
     */

};
