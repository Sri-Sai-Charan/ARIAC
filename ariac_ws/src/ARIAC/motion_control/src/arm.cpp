#include "arm.h"
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
    Arm::Arm(ros::NodeHandle& node) : node_("/ariac/kitting"),
        planning_group_("/ariac/kitting/robot_description"),
        arm_options_("kitting_arm", planning_group_, node_),
        arm_group_(arm_options_)
    {
        ROS_INFO_STREAM("[Arm] constructor called... ");
    }


    /////////////////////////////////////////////////////
    void Arm::init()
    {
        //make sure the planning groups operated in the world frame
        // ROS_INFO_NAMED("init", "Planning frame: %s", arm_group_.getPlanningFrame().c_str());
        //check the name of the end effector
        // ROS_INFO_NAMED("init", "End effector link: %s", arm_group_.getEndEffectorLink().c_str());


        // publishers to directly control the joints without moveit
        arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/kitting/kitting_arm_controller/command", 10);

        // joint state subscribers
        arm_joint_states_subscriber_ =
            node_.subscribe("/ariac/kitting/joint_states", 10, &Arm::arm_joint_states_callback_, this);
        // gripper state subscriber
        arm_gripper_state_subscriber_ = node_.subscribe(
            "/ariac/kitting/arm/gripper/state", 10, &Arm::arm_gripper_state_callback, this);
        // controller state subscribers
        arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/kitting/kitting_arm_controller/state", 10, &Arm::arm_controller_state_callback, this);

        arm_gripper_control_client_ =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/kitting/arm/gripper/control");
        arm_gripper_control_client_.waitForExistence();

        // Preset locations
        // ^^^^^^^^^^^^^^^^
        // Joints for the arm are in this order:
        // - linear_arm_actuator_joint
        // - shoulder_pan_joint
        // - shoulder_lift_joint
        // - elbow_joint
        // - wrist_1_joint
        // - wrist_2_joint
        // - wrist_3_joint

        double linear_arm_actuator_joint{ 0 };
        double shoulder_pan_joint{ 0 };
        double shoulder_lift_joint{ -1.25 };
        double elbow_joint{ 1.74 };
        double wrist_1_joint{ -2.06 };
        double wrist_2_joint{ -1.51 };
        double wrist_3_joint{ 0 };

        //home position
        home_.arm_preset = { 0, 0, -1.25, 1.74, -2.06, -1.51, 0 };
        at_bin1_.arm_preset = { 2.78, 2.14, -1.25, 1.74, -2.06, -1.51, 0 };
        at_agv1_.arm_preset = { 3.84, 2.14, -1.25, 1.74, -2.06, -1.51, 0 };
        // after leaving bin1
        after_bin1_ = before_bin1_;
        // after agv1
        after_agv1_ = before_agv1_;

        // raw pointers are frequently used to refer to the planning group for improved performance.
        // to start, we will create a pointer that references the current robot’s state.
        const moveit::core::JointModelGroup* joint_model_group =
            arm_group_.getCurrentState()->getJointModelGroup("kitting_arm");
        moveit::core::RobotStatePtr current_state = arm_group_.getCurrentState();
        // next get the current set of joint values for the group.
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);
    }


    void Arm::testPreset(const std::vector<ArmPresetLocation>& preset_list) {
        for (auto location : preset_list) {
            goToPresetLocation(location);
            ros::Duration(1.0).sleep();
        }
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
    bool Arm::pickPart(motioncontrol::Part part)
    {
        ros::Duration(3).sleep();
        activateGripper();
        

        // pose of the end effector in the world frame
        geometry_msgs::Pose arm_ee_link_pose = arm_group_.getCurrentPose().pose;

        // pose to go to after grasping a part (lift the arm a little bit)
        geometry_msgs::Pose postGraspPose;
        auto part_init_pose_in_world = part.getInitPoseInWorld();
        part_init_pose_in_world.position.z = part_init_pose_in_world.position.z + 0.09;
        part_init_pose_in_world.orientation.x = arm_ee_link_pose.orientation.x;
        part_init_pose_in_world.orientation.y = arm_ee_link_pose.orientation.y;
        part_init_pose_in_world.orientation.z = arm_ee_link_pose.orientation.z;
        part_init_pose_in_world.orientation.w = arm_ee_link_pose.orientation.w;

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
            arm_group_.setPoseTarget(part_init_pose_in_world);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            arm_group_.plan(plan);
            auto plan_result = arm_group_.execute(plan);//TODO catch the error message

            state = getGripperState();
            // move the arm closer until the object is attached
            while (!state.attached) {
                part_init_pose_in_world.position.z = part_init_pose_in_world.position.z - 0.0005;
                arm_group_.setPoseTarget(part_init_pose_in_world);
                arm_group_.move();
                arm_group_.setPoseTarget(arm_ee_link_pose);
                state = getGripperState();
            }

            ROS_INFO_STREAM("[Gripper] = object attached");
            goToPresetLocation(at_bin1_);
            return true;
        }
        return false;

    }


    /////////////////////////////////////////////////////
    bool Arm::placePart(motioncontrol::Part part)
    {
        auto agv = part.getAGV();
        auto target_pose_in_frame = part.getTargetPoseInFrame();
        // get the target pose of the part in the world frame
        auto part_in_world_frame = motioncontrol::transformToWorldFrame(
            target_pose_in_frame,
            agv,
            "arm");

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

        geometry_msgs::Pose currentPose = arm_group_.getCurrentPose().pose;

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

        auto ee_pose = arm_group_.getCurrentPose().pose;

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
        part_in_world_frame.position.z += 0.2;

        //allow replanning if it fails
        // arm_gantry_group_.allowReplanning(true);
        //Set the allowable error of position (unit: meter) and attitude (unit: radians)
        // arm_gantry_group_.setGoalPositionTolerance(0.001);
        // arm_gantry_group_.setGoalOrientationTolerance(0.01);
        // //Set the maximum speed and acceleration allowed
        // arm_gantry_group_.setMaxAccelerationScalingFactor(0.2);
        // arm_gantry_group_.setMaxVelocityScalingFactor(0.2);

        arm_group_.setPoseTarget(part_in_world_frame);
        arm_group_.move();
        ros::Duration(2.0).sleep();
        deactivateGripper();
        auto state = getGripperState();
        if (state.attached) {
            goToPresetLocation(home_);
            return true;
        }
            
        else
            return false;
        // TODO: check the part was actually placed in the correct pose in the agv
        // and that it is not faulty
    }


    /////////////////////////////////////////////////////
    void Arm::activateGripper()
    {
        nist_gear::VacuumGripperControl srv;
        srv.request.enable = true;
        arm_gripper_control_client_.call(srv);

        ROS_INFO_STREAM("[Arm][activateGripper] DEBUG: srv.response =" << srv.response);
    }

    /////////////////////////////////////////////////////
    void Arm::deactivateGripper()
    {
        nist_gear::VacuumGripperControl srv;
        srv.request.enable = false;
        arm_gripper_control_client_.call(srv);

        ROS_INFO_STREAM("[Arm][deactivateGripper] DEBUG: srv.response =" << srv.response);
    }

    /////////////////////////////////////////////////////
    nist_gear::VacuumGripperState Arm::getGripperState()
    {
        return gripper_state_;
    }

    /////////////////////////////////////////////////////
    void Arm::goToPresetLocation(ArmPresetLocation location)
    {
        joint_group_positions_.at(0) = location.arm_preset.at(0);
        joint_group_positions_.at(1) = location.arm_preset.at(1);
        joint_group_positions_.at(2) = location.arm_preset.at(2);
        joint_group_positions_.at(3) = location.arm_preset.at(3);
        joint_group_positions_.at(4) = location.arm_preset.at(4);
        joint_group_positions_.at(5) = location.arm_preset.at(5);
        joint_group_positions_.at(6) = location.arm_preset.at(6);

        arm_group_.setJointValueTarget(joint_group_positions_);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // check a plan is found first then execute the action
        bool success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
            arm_group_.move();
    }


    ///////////////////////////
    ////// Callback Functions
    ///////////////////////////


    /////////////////////////////////////////////////////
    void Arm::arm_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg)
    {
        gripper_state_ = *gripper_state_msg;
    }

    /////////////////////////////////////////////////////
    void Arm::arm_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
    {
        if (joint_state_msg->position.size() == 0) {
            ROS_ERROR("[Arm][arm_joint_states_callback_] joint_state_msg->position.size() == 0!");
        }
        current_joint_states_ = *joint_state_msg;
    }

    /////////////////////////////////////////////////////
    void Arm::arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
    {
        arm_controller_state_ = *msg;
    }
}//namespace

