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
 * @brief Class for Kitting robot to perform pick, place and trash and complete a kitting shipment
 * 
 */
class KittingRobot {

    public:

    /**
     * @brief Construct a new Kitting Robot object
     * 
     * @param node 
     */
    KittingRobot(ros::NodeHandle& node);

    /**
     * @brief Init function
     * 
     */
    void init(); // initialize the kitting robot class

    // arm predefined locations
    std::vector<double> arm_preset_at_home_conveyor = { 0.10, 1.57, -0.75, 1.51, -2.13, -1.51, 0.0};
    std::vector<double> arm_preset_at_home = { 0.10, 3.15, -0.75, 1.51, -2.13, -1.51, 0.0};
    std::vector<double> arm_preset_at_bins0 = { 3.07, 3.15, -0.75, 1.51, -2.33, -1.51, 0.0};
    std::vector<double> arm_preset_at_bins1 = { -2.88, 3.15, -0.75,  1.51, -2.33, -1.51, 0.0};
    std::vector<double> arm_preset_at_agv1 = { 4.32, 2.64, -0.75, 1.51, -2.33, -1.51, 0.0};
    std::vector<double> arm_preset_at_agv2 = { 1.06, 2.64, -0.75, 1.51, -2.33, -1.51, 0.0};
    std::vector<double> arm_preset_at_agv3 = { -1.63, 2.64, -0.75, 1.51, -2.33, -1.51, 0.0};
    std::vector<double> arm_preset_at_agv4 = { -4.32, 3.40, -0.75, 1.51, -2.33, -1.51, 0.0};
    std::vector<double> arm_preset_at_trash_bin = { 0.10, 3.15, -0.75, 1.51, -2.13, -1.51, 0.0};

    std::vector<double> arm_preset_pre_grasp;
    std::vector<double> arm_preset_post_grasp;
    
    private:
    ros::NodeHandle nh;
    std::string kitting_robot_planing_grp;

    std::vector<double> current_joint_group_positions;
    std::vector<double> joint_arm_positions;

    // gripper attributes
    nist_gear::VacuumGripperState current_gripper_state;

    // msgs to store robot states
    nist_gear::VacuumGripperState kitting_robot_gripper_state;
    sensor_msgs::JointState current_kitting_robot_joint_states;
    control_msgs::JointTrajectoryControllerState kitting_robot_controller_state;

    // kitting robot arm moveit configurations
    moveit::planning_interface::MoveGroupInterface::Options kitting_robot_moveit_options;
    moveit::planning_interface::MoveGroupInterface kitting_robot_moveit_group;

    // Publisher nodes
    ros::Publisher kitting_robot_joint_trajectory_publisher;
    ros::Subscriber kitting_robot_joint_states_subscriber;
    ros::Subscriber kitting_robot_gripper_state_subscriber;
    ros::Subscriber kitting_robot_controller_state_subscriber;

    // Service clients and servers
    ros::ServiceClient kitting_robot_gripper_control_service_client;
    ros::ServiceServer kitting_robot_pick_place_service;
    ros::ServiceServer kitting_robot_dispose_faulty_service;

    /**
     * @brief method to move the kitting robot on the linear joint actuator 
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
     * @brief callback function for the joint states of kitting robot
     * 
     * @param joint_state_msg 
     */
    void kitting_robot_joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg);

    /**
     * @brief callback function for the gripper state of kitting robot
     * 
     * @param vacuum_state_msg 
     */
    void kitting_robot_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& vacuum_state_msg);

    /**
     * @brief callback function for the controller state of kitting robot
     * 
     * @param controller_state_msg 
     */
    void kitting_robot_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& controller_state_msg); 

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
    bool kitting_robot_pick_place_service_callback(group2_rwa4::kitting_part_details::Request &req, group2_rwa4::kitting_part_details::Response &res);
    
    /**
     * @brief service callback function to trash a part using kitting robot
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */
    bool kitting_robot_dispose_faulty_service_callback(group2_rwa4::dispose_faulty_part::Request &req, group2_rwa4::dispose_faulty_part::Response &res);
};

