#ifndef __ARM_H__
#define __ARM_H__

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
// standard library
#include <string>
#include <vector>
#include <array>
#include <cstdarg>
// nist
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>
// custom
#include "utils.h"
#include "part.h"

namespace motioncontrol {


    /**
     * @brief class for the Gantry robot
     *
     */
    class Arm {
        public:
        /**
        * @brief Struct for preset locations
        * @todo Add new preset locations here if needed
        */
        typedef struct ArmPresetLocation {
            std::vector<double> arm_preset;  //9 joints
        } start, bin, agv, grasp;

        Arm(ros::NodeHandle& node);
        /**
         * @brief Initialize the object
         */
        void init();
        bool pickPart(motioncontrol::Part part);
        bool placePart(motioncontrol::Part part);
        //variadic function
        void testPreset(const std::vector<ArmPresetLocation>& preset_list);


        // Send command message to robot controller
        bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
        void goToPresetLocation(ArmPresetLocation location);
        void activateGripper();
        void deactivateGripper();

        nist_gear::VacuumGripperState getGripperState();
        //--preset locations;
        start home_;
        bin at_bin1_, before_bin1_, after_bin1_;
        agv before_agv1_, at_agv1_, after_agv1_;
        grasp pre_grasp_, post_grasp_;

        private:
        std::vector<double> joint_group_positions_;
        std::vector<double> joint_arm_positions_;
        ros::NodeHandle node_;
        std::string planning_group_;
        moveit::planning_interface::MoveGroupInterface::Options arm_options_;
        moveit::planning_interface::MoveGroupInterface arm_group_;
        sensor_msgs::JointState current_joint_states_;
        nist_gear::VacuumGripperState gripper_state_;
        control_msgs::JointTrajectoryControllerState arm_controller_state_;

        // publishers
        ros::Publisher arm_joint_trajectory_publisher_;

        // joint states subscribers
        ros::Subscriber arm_joint_states_subscriber_;
        // gripper state subscriber
        ros::Subscriber arm_gripper_state_subscriber_;
        // controller state subscribers
        ros::Subscriber arm_controller_state_subscriber_;

        // service client
        ros::ServiceClient arm_gripper_control_client_;

        // callbacks
        void arm_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
        void arm_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& msg);
        void arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
    };
}//namespace
#endif
