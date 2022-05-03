#include "gantry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf2/convert.h>
#include "utils.h"
#include <math.h>
#include "part.h"

namespace motioncontrol {
    /////////////////////////////////////////////////////
    Gantry::Gantry(ros::NodeHandle& node) : node_("/ariac/gantry"),
        planning_group_("/ariac/gantry/robot_description"),
        full_gantry_options_("gantry_full", planning_group_, node_),
        arm_gantry_options_("gantry_arm", planning_group_, node_),
        torso_gantry_options_("gantry_torso", planning_group_, node_),
        full_gantry_group_(full_gantry_options_),
        arm_gantry_group_(arm_gantry_options_),
        torso_gantry_group_(torso_gantry_options_)
    {
        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/moveit_visual_markers"));
        ROS_INFO_STREAM("[Gantry] constructor called... ");
    }



    /////////////////////////////////////////////////////
    void Gantry::init()
    {
        //make sure the planning groups operated in the world frame
        // ROS_INFO_NAMED("init", "Planning frame: %s", full_gantry_group_.getPlanningFrame().c_str());
        // ROS_INFO_NAMED("init", "Planning frame: %s", arm_gantry_group_.getPlanningFrame().c_str());
        // ROS_INFO_NAMED("init", "Planning frame: %s", torso_gantry_group_.getPlanningFrame().c_str());

        //check the name of the end effector
        // ROS_INFO_NAMED("init", "End effector link: %s", full_gantry_group_.getEndEffectorLink().c_str());
        // ROS_INFO_NAMED("init", "End effector link: %s", arm_gantry_group_.getEndEffectorLink().c_str());


        // publishers to directly control the joints without moveit
        gantry_arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_arm_controller/command", 10);
        gantry_torso_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);

        // joint state subscribers
        gantry_full_joint_states_subscriber_ =
            node_.subscribe("/ariac/gantry/joint_states", 10, &Gantry::gantry_full_joint_states_callback_, this);
        // gripper state subscriber
        gantry_gripper_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/arm/gripper/state", 10, &Gantry::gantry_gripper_state_callback, this);
        // controller state subscribers
        gantry_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/gantry_controller/state", 10, &Gantry::gantry_controller_state_callback, this);
        gantry_arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/gantry_arm_controller/state", 10, &Gantry::gantry_arm_controller_state_callback, this);

        gantry_gripper_control_client_ =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/arm/gripper/control");
        gantry_gripper_control_client_.waitForExistence();

        // Preset locations
        // ^^^^^^^^^^^^^^^^
        // Joints for the gantry are in this order:
        // For the torso
        // - small_long_joint
        // - torso_rail_joint
        // - torso_base_main_joint
        // For the arm
        // - gantry_arm_shoulder_pan_joint
        // - gantry_arm_shoulder_lift_joint
        // - gantry_arm_elbow_joint
        // - gantry_arm_wrist_1
        // - gantry_arm_wrist_2
        // - gantry_arm_wrist_3
        // For the full robot = torso + arm

 
        home_.gantry_torso_preset = { -3.90, -0.13, -0.02 };
        home_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        //concatenate gantry torso and gantry arm
        home_.gantry_full_preset.insert(home_.gantry_full_preset.begin(), home_.gantry_torso_preset.begin(), home_.gantry_torso_preset.end());
        home_.gantry_full_preset.insert(home_.gantry_full_preset.end(), home_.gantry_arm_preset.begin(), home_.gantry_arm_preset.end());
        // print(home_.gantry_full);

        //safe spot to reach any bin without colliding with anything
        safe_bins_.gantry_torso_preset = { -6.90, -0.13, -0.02 };
        safe_bins_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        safe_bins_.gantry_full_preset.insert(safe_bins_.gantry_full_preset.begin(), safe_bins_.gantry_torso_preset.begin(), safe_bins_.gantry_torso_preset.end());
        safe_bins_.gantry_full_preset.insert(safe_bins_.gantry_full_preset.end(), safe_bins_.gantry_arm_preset.begin(), safe_bins_.gantry_arm_preset.end());

        //at bins 1, 2, 3, 4
        at_bins1234_.gantry_torso_preset = { -1.72, -2.90, -0.02 };
        at_bins1234_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_bins1234_.gantry_full_preset.insert(at_bins1234_.gantry_full_preset.begin(), at_bins1234_.gantry_torso_preset.begin(), at_bins1234_.gantry_torso_preset.end());
        at_bins1234_.gantry_full_preset.insert(at_bins1234_.gantry_full_preset.end(), at_bins1234_.gantry_arm_preset.begin(), at_bins1234_.gantry_arm_preset.end());
        
        // above bin1
        at_bin1_.gantry_torso_preset = { -0.09, -2.45, 0.0 };
        at_bin1_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_bin1_.gantry_full_preset.insert(at_bin1_.gantry_full_preset.begin(), at_bin1_.gantry_torso_preset.begin(), at_bin1_.gantry_torso_preset.end());
        at_bin1_.gantry_full_preset.insert(at_bin1_.gantry_full_preset.end(), at_bin1_.gantry_arm_preset.begin(), at_bin1_.gantry_arm_preset.end());

        // before approaching agv1
        // before_agv1_.gantry_torso = { 0.07, -2.92, -0.02 };
        // before_agv1_.gantry_arm = { 0.07 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        // before_agv1_.gantry_full.insert(before_agv1_.gantry_full.begin(), before_agv1_.gantry_torso.begin(), before_agv1_.gantry_torso.end());
        // before_agv1_.gantry_full.insert(before_agv1_.gantry_full.end(), before_agv1_.gantry_arm.begin(), before_agv1_.gantry_arm.end());

        // above agv1
        //small_long_joint. torso_rail_joint, torso_base_main_joint
        at_agv1_.gantry_torso_preset = { 0.07, -3.73, -0.02 };
        at_agv1_.gantry_arm_preset = { 0.07 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
        at_agv1_.gantry_full_preset.insert(at_agv1_.gantry_full_preset.begin(), at_agv1_.gantry_torso_preset.begin(), at_agv1_.gantry_torso_preset.end());
        at_agv1_.gantry_full_preset.insert(at_agv1_.gantry_full_preset.end(), at_agv1_.gantry_arm_preset.begin(), at_agv1_.gantry_arm_preset.end());


        // raw pointers are frequently used to refer to the planning group for improved performance.
        // to start, we will create a pointer that references the current robot’s state.
        const moveit::core::JointModelGroup* joint_model_group =
            full_gantry_group_.getCurrentState()->getJointModelGroup("gantry_full");
        moveit::core::RobotStatePtr current_state = full_gantry_group_.getCurrentState();
        // next get the current set of joint values for the group.
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);


        const moveit::core::JointModelGroup* joint_arm_group =
            arm_gantry_group_.getCurrentState()->getJointModelGroup("gantry_arm");
        moveit::core::RobotStatePtr current_state_arm = arm_gantry_group_.getCurrentState();
        current_state_arm->copyJointGroupPositions(joint_arm_group, joint_arm_positions_);
    }


    /**
     * @brief Pick up a part from a bin
     *
     * @param part Part to pick up
     * @return true Part was picked up
     * @return false Part was not picked up
     *
     * We use the group full_gantry_group_ to allow the robot more flexibility
     */
    bool Gantry::pickPart(motioncontrol::Part part)
    {
        ros::Duration(3).sleep();
        activateGripper();
        const double GRIPPER_HEIGHT = 0.01;
        const double EPSILON = 0.008; // for the gripper to firmly touch
        ros::Duration(0.5).sleep();

        // pose of the end effector in the world frame
        geometry_msgs::Pose gantry_ee_link_pose = arm_gantry_group_.getCurrentPose().pose;
        tf2::Quaternion  gantry_ee_link_orientation(
            gantry_ee_link_pose.orientation.x,
            gantry_ee_link_pose.orientation.y,
            gantry_ee_link_pose.orientation.z,
            gantry_ee_link_pose.orientation.w
        );

        // pose to go to after grasping a part (lift the arm a little bit)
        geometry_msgs::Pose postGraspPose;
        auto part_init_pose_in_world = part.getInitPoseInWorld();

        part_init_pose_in_world.position.z = part_init_pose_in_world.position.z + 0.09;
        part_init_pose_in_world.orientation.x = gantry_ee_link_pose.orientation.x;
        part_init_pose_in_world.orientation.y = gantry_ee_link_pose.orientation.y;
        part_init_pose_in_world.orientation.z = gantry_ee_link_pose.orientation.z;
        part_init_pose_in_world.orientation.w = gantry_ee_link_pose.orientation.w;

        // activate gripper
        ros::Duration(3).sleep();
        activateGripper();
        auto state = getGripperState();


        if (!state.enabled) {
            int MAX_ATTEMPT = 10;
            for (int i{}; i < MAX_ATTEMPT; i++) {
                ros::Duration(0.5).sleep();
                activateGripper();
                state = getGripperState();
            }
        }

        if (!state.enabled) {
            ROS_FATAL_STREAM("[Gripper] = Could not enable gripper...shutting down");
            ros::shutdown();
        }

        if (state.enabled) {
            ROS_INFO_STREAM("[Gripper] = enabled");
            //--Move arm to part
            full_gantry_group_.setPoseTarget(part_init_pose_in_world);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            full_gantry_group_.plan(plan);
            auto plan_result = full_gantry_group_.execute(plan);//TODO catch the error message

            state = getGripperState();
            // move the arm closer until the object is attached
            while (!state.attached) {
                part_init_pose_in_world.position.z = part_init_pose_in_world.position.z - 0.0005;
                full_gantry_group_.setPoseTarget(part_init_pose_in_world);
                full_gantry_group_.move();
                full_gantry_group_.setPoseTarget(gantry_ee_link_pose);
                state = getGripperState();
            }

            ROS_INFO_STREAM("[Gripper] = object attached");
            // ros::Duration(1.0).sleep();
            postGraspPose = full_gantry_group_.getCurrentPose().pose;
            postGraspPose.position.z = postGraspPose.position.z + 0.1;
            //--Move arm to previous position
            full_gantry_group_.setPoseTarget(postGraspPose);
            full_gantry_group_.move();
            ros::Duration(2.0).sleep();
            full_gantry_group_.setPoseTarget(gantry_ee_link_pose);
            full_gantry_group_.move();
            return true;
        }
        return false;
    }


    /////////////////////////////////////////////////////
    bool Gantry::placePart(motioncontrol::Part part)
    {
        auto agv = part.getAGV();
        auto target_pose_in_frame = part.getTargetPoseInFrame();
        // get the target pose of the part in the world frame
        auto part_in_world_frame = motioncontrol::transformToWorldFrame(
            target_pose_in_frame,
            agv,
            "gantry");

        // complete our struct instance
        geometry_msgs::Pose target_pose_in_world;
        target_pose_in_world.position.x = part_in_world_frame.position.x;
        target_pose_in_world.position.y = part_in_world_frame.position.y;
        target_pose_in_world.position.z = part_in_world_frame.position.z;
        target_pose_in_world.orientation.x = part_in_world_frame.orientation.x;
        target_pose_in_world.orientation.y = part_in_world_frame.orientation.y;
        target_pose_in_world.orientation.z = part_in_world_frame.orientation.z;
        target_pose_in_world.orientation.w = part_in_world_frame.orientation.w;
        motioncontrol::print(target_pose_in_world);
        part.setTargetPoseInWorld(target_pose_in_world);

        geometry_msgs::Pose currentPose = full_gantry_group_.getCurrentPose().pose;

        //TODO: Consider other agvs
        if (agv == "agv1") {
            goToPresetLocation(at_agv1_);
        }


        ROS_INFO("Target World Position: %f, %f, %f",
            part_in_world_frame.position.x,
            part_in_world_frame.position.y,
            part_in_world_frame.position.z);

        ROS_INFO("Target World Orientation: %f, %f, %f, %f",
            part_in_world_frame.orientation.x,
            part_in_world_frame.orientation.y,
            part_in_world_frame.orientation.z,
            part_in_world_frame.orientation.w);

        auto ee_pose = arm_gantry_group_.getCurrentPose().pose;

        tf2::Quaternion q_current(
            ee_pose.orientation.x,
            ee_pose.orientation.y,
            ee_pose.orientation.z,
            ee_pose.orientation.w);

        auto part_init_pose_in_world = part.getInitPoseInWorld();
        // orientation of the part in the bin, in world frame
        tf2::Quaternion q_init_part(
            part_init_pose_in_world.orientation.x,
            part_init_pose_in_world.orientation.y,
            part_init_pose_in_world.orientation.z,
            part_init_pose_in_world.orientation.w);
        // orientation of the part in the tray, in world frame
        tf2::Quaternion q_target_part(
            part_in_world_frame.orientation.x,
            part_in_world_frame.orientation.y,
            part_in_world_frame.orientation.z,
            part_in_world_frame.orientation.w);

        // relative rotation between init and target
        tf2::Quaternion q_rot = q_target_part * q_init_part.inverse();
        // apply this rotation to the current gripper rotation
        tf2::Quaternion q_rslt = q_rot * q_current;
        q_rslt.normalize();

        // orientation of the gripper when placing the part in the tray
        part_in_world_frame.orientation.x = q_rslt.x();
        part_in_world_frame.orientation.y = q_rslt.y();
        part_in_world_frame.orientation.z = q_rslt.z();
        part_in_world_frame.orientation.w = q_rslt.w();
        part_in_world_frame.position.z += 0.1;

        //allow replanning if it fails
        // arm_gantry_group_.allowReplanning(true);
        //Set the allowable error of position (unit: meter) and attitude (unit: radians)
        // arm_gantry_group_.setGoalPositionTolerance(0.001);
        // arm_gantry_group_.setGoalOrientationTolerance(0.01);
        // //Set the maximum speed and acceleration allowed
        // arm_gantry_group_.setMaxAccelerationScalingFactor(0.2);
        // arm_gantry_group_.setMaxVelocityScalingFactor(0.2);

        arm_gantry_group_.setPoseTarget(part_in_world_frame);
        arm_gantry_group_.move();
        ros::Duration(2.0).sleep();
        deactivateGripper();
        auto state = getGripperState();
        if (state.attached)
            return true;
        else
            return false;
        // TODO: check the part was actually placed in the correct pose in the agv
        // and that it is not faulty
    }


    /////////////////////////////////////////////////////
    void Gantry::activateGripper()
    {
        nist_gear::VacuumGripperControl srv;
        srv.request.enable = true;
        gantry_gripper_control_client_.call(srv);

        ROS_INFO_STREAM("[Gantry][activateGantryGripper] DEBUG: srv.response =" << srv.response);
    }

    /////////////////////////////////////////////////////
    void Gantry::deactivateGripper()
    {
        nist_gear::VacuumGripperControl srv;
        srv.request.enable = false;
        gantry_gripper_control_client_.call(srv);

        ROS_INFO_STREAM("[Gantry][deactivateGantryGripper] DEBUG: srv.response =" << srv.response);
    }

    /////////////////////////////////////////////////////
    nist_gear::VacuumGripperState Gantry::getGripperState()
    {
        return gantry_gripper_state_;
    }

    /////////////////////////////////////////////////////
    void Gantry::goToPresetLocation(GantryPresetLocation location, bool full_robot)
    {
        if (full_robot) {
            // gantry torso
            joint_group_positions_.at(0) = location.gantry_torso_preset.at(0);
            joint_group_positions_.at(1) = location.gantry_torso_preset.at(1);
            joint_group_positions_.at(2) = location.gantry_torso_preset.at(2);
            // gantry arm
            joint_group_positions_.at(3) = location.gantry_arm_preset.at(0);
            joint_group_positions_.at(4) = location.gantry_arm_preset.at(1);
            joint_group_positions_.at(5) = location.gantry_arm_preset.at(2);
            joint_group_positions_.at(6) = location.gantry_arm_preset.at(3);
            joint_group_positions_.at(7) = location.gantry_arm_preset.at(4);
            joint_group_positions_.at(8) = location.gantry_arm_preset.at(5);

            full_gantry_group_.setJointValueTarget(joint_group_positions_);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            // check a plan is found first then execute the action
            bool success = (full_gantry_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success)
                full_gantry_group_.move();
        }
        else {
            // gantry torso
            joint_group_positions_.at(0) = location.gantry_torso_preset.at(0);
            joint_group_positions_.at(1) = location.gantry_torso_preset.at(1);
            joint_group_positions_.at(2) = location.gantry_torso_preset.at(2);

            torso_gantry_group_.setJointValueTarget(joint_group_positions_);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            // check a plan is found first then execute the action
            bool success = (torso_gantry_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success)
                torso_gantry_group_.move();
        }

    }


    ///////////////////////////
    ////// Callback Functions
    ///////////////////////////


    /////////////////////////////////////////////////////
    void Gantry::gantry_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg)
    {
        gantry_gripper_state_ = *gripper_state_msg;
    }

    /////////////////////////////////////////////////////
    void Gantry::gantry_full_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
    {
        if (joint_state_msg->position.size() == 0) {
            ROS_ERROR("[Gantry][gantry_full_joint_states_callback_] joint_state_msg->position.size() == 0!");
        }
        current_joint_states_ = *joint_state_msg;
    }

    /////////////////////////////////////////////////////
    void Gantry::gantry_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
    {
        gantry_arm_controller_state_ = *msg;
    }

    /////////////////////////////////////////////////////
    void Gantry::gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
    {
        gantry_torso_controller_state_ = *msg;
    }
}//namespace

