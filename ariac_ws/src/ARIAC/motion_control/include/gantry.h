#ifndef __GANTRY_H__
#define __GANTRY_H__

//ros
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/String.h>
//nist
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>
//moveit
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
//standard library
#include <string>
#include <vector>
#include <array>


#include "utils.h"
#include "part.h"


namespace motioncontrol {


    /**
     * @brief class for the Gantry robot
     *
     */
    class Gantry {

        public:
        /**
     * @brief Struct for preset locations
     * @todo Add new preset locations here if needed
     *
     */
        typedef struct GantryPresetLocation {
            std::vector<double> gantry_full_preset;  //3 joints
            std::vector<double> gantry_torso_preset; //6 joints
            std::vector<double> gantry_arm_preset;   //9 joints
        } start, bin, agv, grasp;

        Gantry(ros::NodeHandle& node);
        /**
         * @brief Initialize the object
         *
         * Set the different groups.
         *
         */
        void init();
        bool pickPart(motioncontrol::Part part);
        bool placePart(motioncontrol::Part part);


        // Send command message to robot controller
        bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
        void goToPresetLocation(GantryPresetLocation location, bool full_robot=true);
        void activateGripper();
        void deactivateGripper();
        nist_gear::VacuumGripperState getGripperState();
        //--preset locations;
        start home_;
        bin at_bin1_, safe_bins_, at_bins1234_;
        agv at_agv1_;
        grasp pre_grasp_, post_grasp_;

        private:
        std::vector<double> joint_group_positions_;
        std::vector<double> joint_arm_positions_;
        ros::NodeHandle node_;
        std::string planning_group_;
        moveit::planning_interface::MoveGroupInterface::Options full_gantry_options_;
        moveit::planning_interface::MoveGroupInterface::Options arm_gantry_options_;
        moveit::planning_interface::MoveGroupInterface::Options torso_gantry_options_;
        moveit::planning_interface::MoveGroupInterface full_gantry_group_;
        moveit::planning_interface::MoveGroupInterface arm_gantry_group_;
        moveit::planning_interface::MoveGroupInterface torso_gantry_group_;
        sensor_msgs::JointState current_joint_states_;
        nist_gear::VacuumGripperState gantry_gripper_state_;
        control_msgs::JointTrajectoryControllerState gantry_torso_controller_state_;
        control_msgs::JointTrajectoryControllerState gantry_arm_controller_state_;

        // publishers
        ros::Publisher gantry_torso_joint_trajectory_publisher_;
        ros::Publisher gantry_arm_joint_trajectory_publisher_;

        // joint states subscribers
        ros::Subscriber gantry_full_joint_states_subscriber_;
        // gripper state subscriber
        ros::Subscriber gantry_gripper_state_subscriber_;
        // controller state subscribers
        ros::Subscriber gantry_controller_state_subscriber_;
        ros::Subscriber gantry_arm_controller_state_subscriber_;

        // service client
        ros::ServiceClient gantry_gripper_control_client_;

        // For visualizing things in rviz
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

        // callbacks
        void gantry_full_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
        void gantry_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& msg);
        void gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
        void gantry_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
    };
} //namespace

#endif
