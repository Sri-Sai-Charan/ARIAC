// ###################################################################
// ################### Importing Necessary Libraries #################
// ###################################################################

#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Model.h>
#include <group2_rwa4/list_all_parts.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

//###################################################################

namespace sensor_array{
    
    
    /**
     * @brief Class for sensors on conveyor
     * 
     */
    class ConveyorSensors
    {
    private:
        ros::NodeHandle nh;
        std::vector<nist_gear::Model> conveyor_parts;

        // Subscribers
        ros::Subscriber logical_camera_0_sub;
        ros::Subscriber laser_profiler_0_sub;
        ros::Subscriber proximity_sensor_0_sub;
        ros::Subscriber breakbeam_0_sub;

        // Services
        ros::ServiceServer get_all_parts_on_conveyor_service;



    public:

        /**
         * @brief Construct a new Conveyor Sensors object
         * 
         * @param node_handler 
         */
        ConveyorSensors(ros::NodeHandle &node_handler);

        /**
         * @brief init function
         * 
         */
        void init();

        /**
         * @brief callback function for logical camera
         * 
         * @param logicam_msg 
         */
        void logical_camera_0_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);
        // void laser_profiler_0_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);
        // void proximity_sensor_0_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);
        // void breakbeam_0_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);

        /**
         * @brief Get the all parts on conveyor service callback object
         * 
         * @param req 
         * @param res 
         * @return true 
         * @return false 
         */
        bool get_all_parts_on_conveyor_service_callback(group2_rwa4::list_all_parts::Request &req, group2_rwa4::list_all_parts::Response &res);

    };
    
// ############################################################


    /**
     * @brief Class for sensors on bins
     * 
     */
    class BinSensors
    {
    private:
        ros::NodeHandle nh;
        std::vector<nist_gear::Model> bins_parts;

        // Subscribers
        ros::Subscriber logicam_bin1_sub;  
        ros::Subscriber logicam_bin0_sub;

        int bins0_parts_count; 
        std::vector<nist_gear::Model> bins0_parts;
        int bins1_parts_count; 
        std::vector<nist_gear::Model> bins1_parts;
        std::vector<geometry_msgs::Pose> all_parts;

        ros::ServiceServer get_all_parts_on_bins_service;

    public:

        /**
         * @brief Construct a new Bin Sensors object
         * 
         * @param node_handler 
         */
        BinSensors(ros::NodeHandle &node_handler);

        /**
         * @brief init function
         * 
         */
        void init();

        /**
         * @brief callback function for logical camera on bins0
         * 
         * @param logicam_msg 
         */
        void logicam_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);

        /**
         * @brief callback function for logical camera on bins1
         * 
         * @param logicam_msg 
         */
        void logicam_bins1_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);

        /**
         * @brief Get the all parts on bins service callback object
         * 
         * @param req 
         * @param res 
         * @return true 
         * @return false 
         */
        bool get_all_parts_on_bins_service_callback(group2_rwa4::list_all_parts::Request &req, group2_rwa4::list_all_parts::Response &res);
    };

    

// ############################################################# 
    
    /**
     * @brief Class for sensors on AGVs
     * 
     */
    class AgvSensors
    {
    private:
        ros::NodeHandle nh;
        std::vector<nist_gear::Model> agv1_parts;
        std::vector<nist_gear::Model> agv2_parts;
        std::vector<nist_gear::Model> agv3_parts;
        std::vector<nist_gear::Model> agv4_parts;

        std::vector<nist_gear::Model> faulty_agv1_parts;
        std::vector<nist_gear::Model> faulty_agv2_parts;
        std::vector<nist_gear::Model> faulty_agv3_parts;
        std::vector<nist_gear::Model> faulty_agv4_parts;
        int agv1_parts_count;
        int agv2_parts_count;
        int agv3_parts_count;
        int agv4_parts_count;

        // Subscribers
        ros::Subscriber logical_camera_3_sub;
        ros::Subscriber logical_camera_4_sub;
        ros::Subscriber logical_camera_5_sub;
        ros::Subscriber logical_camera_6_sub;
        ros::Subscriber quality_control_sensor_1_sub;
        ros::Subscriber quality_control_sensor_2_sub;
        ros::Subscriber quality_control_sensor_3_sub;
        ros::Subscriber quality_control_sensor_4_sub;

        // Services
        ros::ServiceServer get_all_parts_on_agv_logical_cam_service;
        ros::ServiceServer get_all_parts_on_agv_qc_sensor_service;


        
    public:

        /**
         * @brief Construct a new Agv Sensors object
         * 
         * @param node_handler 
         */
        AgvSensors(ros::NodeHandle &node_handler);

        /**
         * @brief init function
         * 
         */
        void init();

        /**
         * @brief callback function for QC sensor 1
         * 
         * @param msg 
         */
        void quality_control_sensor_1_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);

        /**
         * @brief callback function for QC sensor 2
         * 
         * @param msg 
         */
        void quality_control_sensor_2_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);

        /**
         * @brief callback function for QC sensor 3
         * 
         * @param msg 
         */
        void quality_control_sensor_3_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);

        /**
         * @brief callback function for QC sensor 4
         * 
         * @param msg 
         */
        void quality_control_sensor_4_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);

        /**
         * @brief callback function for logical camera sensor on AGV1
         * 
         * @param msg 
         */
        void logical_camera_3_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);

        /**
         * @brief callback function for logical camera sensor on AGV2
         * 
         * @param msg 
         */
        void logical_camera_4_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);

        /**
         * @brief callback function for logical camera sensor on AGV3
         * 
         * @param msg 
         */
        void logical_camera_5_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);

        /**
         * @brief callback function for logical camera sensor on AGV4
         * 
         * @param msg 
         */
        void logical_camera_6_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);

        /**
         * @brief Get the parts on agv1 service object
         * 
         * @param req 
         * @param res 
         * @return true 
         * @return false 
         */
        bool get_parts_on_agv1_service(group2_rwa4::list_all_parts::Request &req, group2_rwa4::list_all_parts::Response &res);

        /**
         * @brief Get the parts on agv2 service object
         * 
         * @param req 
         * @param res 
         * @return true 
         * @return false 
         */
        bool get_parts_on_agv2_service(group2_rwa4::list_all_parts::Request &req, group2_rwa4::list_all_parts::Response &res);

        /**
         * @brief Get the parts on agv3 service object
         * 
         * @param req 
         * @param res 
         * @return true 
         * @return false 
         */
        bool get_parts_on_agv3_service(group2_rwa4::list_all_parts::Request &req, group2_rwa4::list_all_parts::Response &res);

        /**
         * @brief Get the parts on agv4 service object
         * 
         * @param req 
         * @param res 
         * @return true 
         * @return false 
         */
        bool get_parts_on_agv4_service(group2_rwa4::list_all_parts::Request &req, group2_rwa4::list_all_parts::Response &res);

        /**
         * @brief Get the all parts on agv logical cam service callback object
         * 
         * @param req 
         * @param res 
         * @return true 
         * @return false 
         */
        bool get_all_parts_on_agv_logical_cam_service_callback(group2_rwa4::list_all_parts::Request &req, group2_rwa4::list_all_parts::Response &res);

        /**
         * @brief Get the all parts on agv qc sensor service callback object
         * 
         * @param req 
         * @param res 
         * @return true 
         * @return false 
         */
        bool get_all_parts_on_agv_qc_sensor_service_callback(group2_rwa4::list_all_parts::Request &req, group2_rwa4::list_all_parts::Response &res);




    };
    
    

    // class AssemblySensors
    // {
    // private:
    //     ros::NodeHandle nh;
    //     std::vector<nist_gear::Model> as_parts;
    // public:
    //     AssemblySensors(ros::NodeHandle &node_handler);
    // };
    /**
     * @brief function to transform pose to world frame
     * 
     * @param part_pose 
     * @param logicam_frame 
     * @return geometry_msgs::Pose 
     */
    geometry_msgs::Pose transform_to_world_frame(const geometry_msgs::Pose& part_pose, std::string logicam_frame);
    
}