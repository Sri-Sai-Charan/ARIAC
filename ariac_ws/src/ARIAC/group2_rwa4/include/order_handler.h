// ###################################################################
// ################### Importing Necessary Libraries #################
// ###################################################################

#include <vector>
#include <string>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>


#include <nist_gear/Order.h>
#include <nist_gear/Model.h>
#include <group2_rwa4/list_order_details.h>
#include "group2_rwa4/IncompleteOrder.h"
#include <group2_rwa4/Product.h>
#include <group2_rwa4/order_kitting_shipment_details.h>
#include <group2_rwa4/KittingShipment.h>
#include <group2_rwa4/order_completion_status.h>
#include <group2_rwa4/Task.h>
#include <group2_rwa4/check_exists.h>
#include <group2_rwa4/check_agv_faulty_parts.h>
#include <group2_rwa4/agv_details_at_as.h>
#include <group2_rwa4/assembly_task.h>
#include <nist_gear/AGVToAssemblyStation.h>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf2/convert.h>

// ##################################################################


/**
 * @brief A Class 
 * 
 */
class OrderHandler{

    private:    
    ros::NodeHandle nh;

    ros::Subscriber orders_sub;
    ros::Publisher task_pub;
    ros::Publisher assembly_task_pub;
    ros::ServiceServer order_details_service; 
    ros::ServiceServer order_kitting_shipment_details_service;
    ros::ServiceServer order_completion_status_service;
    ros::ServiceServer task_completed_srv;
    ros::ServiceServer dispose_faulty_srv;
    ros::ServiceServer create_conveyor_task_srv;
    ros::ServiceServer assembly_task_srv;

    ros::ServiceClient logicam_bins0_client;
    ros::ServiceClient logicam_bins1_client;
    // ros::ServiceClient agv_details_at_as_client;

    std::vector<nist_gear::Order> incomplete_orders;
    std::vector<nist_gear::Order> completed_orders;

    group2_rwa4::IncompleteOrder incomplete_order_details_msg;
    bool high_priority_order_recieved;

    std::string current_order_id;
    std::string order_kitting_shipment_type;
    std::string order_kitting_agv_id;
    std::string order_kitting_destination_id;
    std::vector<nist_gear::Model> order_kitting_products;

    std::string order_assembly_station_id;
    std::string order_assembly_shipment_type;
    std::vector<nist_gear::Model> order_shipment_products;

    group2_rwa4::Product product;
    group2_rwa4::KittingShipment kitting_shipment;

    int kitting_task_id = 0;
    std::vector<group2_rwa4::Task> kitting_task_queue;

    int assembly_task_id = 0;
    std::vector<group2_rwa4::Task> assembly_task_queue;

    geometry_msgs::Pose conveyor_pose;
    
    public:

    /**
     * @brief Construct a new Order Handler object
     * 
     * @param nodehandler 
     */
    OrderHandler(ros::NodeHandle &nodehandler);

    /**
     * @brief init function
     * 
     */
    void init();

    /**
     * @brief Get the recent order
     * 
     * @return nist_gear::Order 
     */
    nist_gear::Order get_recent_order();

     /**
     * @brief Get the completed order count
     * 
     * @return int 
     */
    int get_completed_order_count();

    /**
     * @brief Get the incomplete order count
     * 
     * @return int 
     */
    int get_incomplete_order_count();

    /**
     * @brief Func to process the order recieved
     * 
     * @param order 
     */
    void process_orders(const nist_gear::Order::ConstPtr &order);

    /**
     * @brief Service Callback Func to get the order details
     * 
     * @param req 
     * @param res 
     * @return nist_gear::Order 
     */
    bool order_details_service_callback(group2_rwa4::list_order_details::Request &req, group2_rwa4::list_order_details::Response &res);

    /**
     * @brief Service Callback Func to get the kitting shipment details in an order
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */
    bool order_kitting_shipment_details_service_callback(group2_rwa4::order_kitting_shipment_details::Request &req, group2_rwa4::order_kitting_shipment_details::Response &res);

    bool order_completion_status_service_callback(group2_rwa4::order_completion_status::Request &req, group2_rwa4::order_completion_status::Response &res);

    void update_kitting_task_queue();

    void update_assembly_task_queue();

    geometry_msgs::Pose get_part_target_pose(const geometry_msgs::Pose& target_pose, std::string target_frame, std::string child_frame_id); 

    void publish_task();

    bool task_completed_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool dispose_faulty_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    void check_agv_ready();

    void submit_agv(std::string agv_id, std::string assembly_station, std::string shipment_type);

    bool create_conveyor_task_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    void conveyor_part_callback(const geometry_msgs::Pose& part_pose);

    bool get_next_task_srv_callback(group2_rwa4::assembly_task::Request &req, group2_rwa4::assembly_task::Response &res);


};