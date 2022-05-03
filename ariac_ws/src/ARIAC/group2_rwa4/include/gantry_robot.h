// ###################################################################
// ################### Importing Necessary Libraries #################
// ###################################################################

#include <ros/ros.h>
#include <std_msgs/String.h>

// include messages to store robot configurations
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// nist
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

// standard library
#include <string>
#include <vector>
#include <array>
#include <cstdarg>

#include "utils.h"
#include "part_2.h"


// group2 msg imports
#include <group2_rwa4/KittingLocation.h>
#include <group2_rwa4/kitting_part_details.h>
#include <group2_rwa4/dispose_faulty_part.h>

// ####################################################################


/**
 * @brief Class for Gantry robot to perform pick, place, trash and complete a kitting and assembly shipments
 * 
 */
class GantryRobot {

    public:

    /**
     * @brief Construct a new Gantry Robot object
     * 
     * @param node 
     */
    GantryRobot(ros::NodeHandle& node);

    /**
     * @brief Init function
     * 
     */
    void init(); // initialize the kitting robot class

    // gantry kitting predefined locations
    // std::vector<double> arm_preset_at_home =   {-2.87, -1.6, 0 , 1.73, -1.00, -0.05, -0.72, 1.54, 0.83 }; // {Torso(3), ARM(6)}
    // std::vector<double> arm_preset_at_bins0 =  {-2.87, -1.6, 0 , 1.73, -1.00, -0.05, -0.72, 1.54, 0.83 };
    // std::vector<double> arm_preset_at_bins1 =  {-2.87, -1.6, 0 , 1.73, -1.00, -0.05, -0.72, 1.54, 0.83 };
    // std::vector<double> arm_preset_at_agv1 =   {-1.01, -2.48, 3.69, 0.68, -0.88, 0, 3.38, 1.6, 0 };
    // std::vector<double> arm_preset_at_agv2 =   {-1.01, -0.85, 2.43, 0.68, -0.88, 0, 3.38, 1.6, 0 };
    // std::vector<double> arm_preset_at_agv3 =   {-1.01, -2.48,-2.43, 0.68, -0.88, 0, 3.38, 1.6, 0 };
    // std::vector<double> arm_preset_at_agv4 =   {-1.01, -0.85,-3.69, 0.68, -0.88, 0, 3.38, 1.6, 0 };
////////////////////////////////////////////////////////////////////
    std::vector<double> test =   {-3.63, 0.0, 0.0 ,0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };

    std::vector<double> arm_preset_at_world_center =   {-5, 0, 0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_center_1 =   {-2.5, 0, 0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_center_2 =   {-7, 0, 0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };


    std::vector<double> arm_preset_at_home =   {-2.00, -1.60, 0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };

    std::vector<double> arm_preset_at_bins_left =  {-2.0, -2.9, 0.0 , 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83};
    std::vector<double> arm_preset_at_bins2 =  {-1.23, -3, 0.0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_bins3 =  {-1.2, -2.3, -1.5708 ,0.0, -0.785398, 1.74533, -0.785398, 1.5708, 0.0 };

    std::vector<double> arm_preset_at_bins_right =  {-2.00, 2.88, 0.0  , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_bins6 =  {-1.23, 3.04, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_bins7 =  {-1.23, 3.76, 0.0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };

    std::vector<double> arm_preset_at_agv1 =   {-1.6, -3.88, 0.0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_agv2 =   {-1.6, -1.70, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_agv3 =   {-1.6, 2.08, 0.0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_agv4 =   {-1.6, 4.14, 0.0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };

    std::vector<double> arm_preset_at_pre_as1 =   {-2.00, -2.88, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_pre_as2 =   {-2.00, 2.88, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_pre_as3 =   {-8.00, -2.88, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_pre_as4 =   {-8.00, 2.88, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };

    std::vector<double> arm_preset_at_as1 =   {-3.9, -3.15, -4.71239, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_as2 =   {-8.9, -3.15, -4.71239, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_as3 =   {-3.9, 2.79, -4.71239, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_as4 =   {-8.9, 2.79, -4.71239, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };

    std::vector<double> arm_preset_at_as1_agv1 =   {-2.59, -4.14, 0.0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_as1_agv2 =   {-2.59, -1.9, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_as2_agv1 =   {-7.83, -4.14, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_as2_agv2 =   {-7.83, -1.9, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_as3_agv3 =   {-2.59, 1.9, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_as3_agv4 =   {-2.59, 4.14, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_as4_agv3 =   {-7.83, 1.9, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_as4_agv4 =   {-7.83, 4.14, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };

    // typedef struct GantryPresetLocation {
    //         std::vector<double> gantry_full_preset;  //3 joints
    //         std::vector<double> gantry_torso_preset; //6 joints
    //         std::vector<double> gantry_arm_preset;   //9 joints
    //     } start, bin, agv, grasp;
    
    // start home_;
    // bin atbin1, safebins, atbins1234;
    // agv atagv1;
    // grasp pregrasp, postgrasp;


    // gantry assembly predefined locations
    std::vector<double> arm_preset_pre_grasp;
    std::vector<double> arm_preset_post_grasp;

    

    private:
    ros::NodeHandle nh;
    std::string gantry_robot_planing_grp;

    std::vector<double> current_joint_group_positions;
    std::vector<double> joint_arm_positions_;

    // gripper attributes
    nist_gear::VacuumGripperState current_gripper_state;

    // msgs to store robot states
    nist_gear::VacuumGripperState gantry_robot_gripper_state;
    sensor_msgs::JointState current_gantry_robot_joint_states;
    control_msgs::JointTrajectoryControllerState gantry_robot_controller_state;
    control_msgs::JointTrajectoryControllerState gantry_arm_controller_state_;

    // gantry robot arm moveit configurations
    moveit::planning_interface::MoveGroupInterface::Options gantry_robot_moveit_options;
    moveit::planning_interface::MoveGroupInterface::Options gantry_robot_arm_moveit_options;
    moveit::planning_interface::MoveGroupInterface gantry_robot_moveit_group;
    moveit::planning_interface::MoveGroupInterface gantry_robot_arm_moveit_group;

    // Publisher nodes
    ros::Publisher gantry_robot_joint_trajectory_publisher;
    ros::Publisher gantry_robot_arm_joint_trajectory_publisher;
    ros::Subscriber gantry_robot_joint_states_subscriber;
    ros::Subscriber gantry_robot_gripper_state_subscriber;
    ros::Subscriber gantry_robot_controller_state_subscriber;
    ros::Subscriber gantry_arm_controller_state_subscriber;

    // Service clients and servers
    ros::ServiceClient gantry_robot_gripper_control_service_client;
    ros::ServiceServer gantry_robot_pick_place_service;
    ros::Subscriber gantry_arm_controller_state_subscriber_;
    // ros::ServiceServer gantry_robot_dispose_faulty_service;

    /**
     * @brief method to move the gantry robot on the linear joint actuator 
     * 
     * @param location 
     */
    void go_to_location(std::vector<double> location);

//################# gripper methods ############# 

    /**
     * @brief method to activate the gripper for picking up parts
     * 
     */
    void activate_gripper();

    /**
     * @brief method to de-activate the gripper for placing and trashing parts
     * 
     */
    void deactivate_gripper();

    /**
     * @brief method to get the gripper state object
     * 
     * @return nist_gear::VacuumGripperState 
     */
    nist_gear::VacuumGripperState get_gripper_state();

    /**
     * @brief method to pick up a part
     * 
     * @param part_pose 
     * @return true 
     * @return false 
     */
    bool pickup_part(geometry_msgs::Pose part_pose, double part_offset);

    /**
     * @brief methd to place the part
     * 
     * @param target_pose 
     * @return true 
     * @return false 
     */
    bool place_part(geometry_msgs::Pose target_pose); 

//############## callback functions ############# 

    /**
     * @brief callback function for the joint states of gantry robot
     * 
     * @param joint_state_msg 
     */
    void gantry_robot_joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg);

    /**
     * @brief callback function for the gripper state of gantry robot
     * 
     * @param vacuum_state_msg 
     */
    void gantry_robot_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& vacuum_state_msg);

    /**
     * @brief callback function for the controller state of gantry robot
     * 
     * @param controller_state_msg 
     */
    void gantry_robot_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& controller_state_msg); 

    /**
     * @brief callback function for the controller state of gantry robot
     * 
     * @param controller_state_msg 
     */
    void gantry_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& controller_state_msg); 

    /**
     * @brief subscriber callback function to perform kitting 
     * 
     * @param kitting_location 
     * @return true 
     * @return false 
     */
    bool perform_kitting_sub_callback(const group2_rwa4::KittingLocation kitting_location);

    /**
     * @brief service callback function for picking and placing using the kitting robot
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */
    bool gantry_robot_pick_place_service_callback(group2_rwa4::kitting_part_details::Request &req, group2_rwa4::kitting_part_details::Response &res);
    
    /**
     * @brief service callback function to trash a part using gantry robot
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */
    bool gantry_robot_dispose_faulty_service_callback(group2_rwa4::dispose_faulty_part::Request &req, group2_rwa4::dispose_faulty_part::Response &res);
};

