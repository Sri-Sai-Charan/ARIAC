#include "kitting_robot.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf2/convert.h>


// This passes the parameters necessary for the planning group in the rviz gui
KittingRobot::KittingRobot(ros::NodeHandle& node) : 
    nh("/ariac/kitting"),
    kitting_robot_planing_grp("/ariac/kitting/robot_description"),
    kitting_robot_moveit_options("kitting_arm", kitting_robot_planing_grp, nh),
    kitting_robot_moveit_group(kitting_robot_moveit_options)
{
    // nh = node;
    init();
}

void KittingRobot::init()
{
    ROS_INFO("KittingRobot initialized :)");
    kitting_robot_joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/kitting/kitting_arm_controller/command", 10);

    // joint state subscribers
    kitting_robot_joint_states_subscriber = nh.subscribe("/ariac/kitting/joint_states", 10, &KittingRobot::kitting_robot_joint_states_callback, this);

    // gripper state subscriber
    kitting_robot_gripper_state_subscriber = nh.subscribe("/ariac/kitting/arm/gripper/state", 10, &KittingRobot::kitting_robot_gripper_state_callback, this);
    
    // controller state subscribers
    kitting_robot_controller_state_subscriber = nh.subscribe("/ariac/kitting/kitting_arm_controller/state", 10, &KittingRobot::kitting_robot_controller_state_callback, this);

    kitting_robot_gripper_control_service_client = nh.serviceClient<nist_gear::VacuumGripperControl>("/ariac/kitting/arm/gripper/control");
    kitting_robot_gripper_control_service_client.waitForExistence(); // wait till the service starts

    kitting_robot_pick_place_service = nh.advertiseService("/group2/kitting_robot_pick_and_place", &KittingRobot::kitting_robot_pick_place_service_callback, this);

    kitting_robot_dispose_faulty_service = nh.advertiseService("/group2/kitting_dispose_faulty", &KittingRobot::kitting_robot_dispose_faulty_service_callback, this);


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

  
    const moveit::core::JointModelGroup* kitting_robot_joint_model_group = kitting_robot_moveit_group.getCurrentState()->getJointModelGroup("kitting_arm");
    moveit::core::RobotStatePtr kitting_robot_current_state = kitting_robot_moveit_group.getCurrentState();
    kitting_robot_current_state->copyJointGroupPositions(kitting_robot_joint_model_group, current_joint_group_positions);
}

void KittingRobot::kitting_robot_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& vacuum_state_msg)
{   
    kitting_robot_gripper_state = *vacuum_state_msg;
}

void KittingRobot::kitting_robot_joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
{
    if (joint_state_msg->position.size() == 0) {
        ROS_ERROR("[Arm][arm_joint_states_callback_] joint_state_msg->position.size() == 0!");
    }
    current_kitting_robot_joint_states = *joint_state_msg;
}

void KittingRobot::kitting_robot_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& controller_state_msg)
{
    kitting_robot_controller_state = *controller_state_msg;
}

void KittingRobot::go_to_location(std::vector<double> location)
{
    current_joint_group_positions.at(0) = location.at(0);
    current_joint_group_positions.at(1) = location.at(1);
    current_joint_group_positions.at(2) = location.at(2);
    current_joint_group_positions.at(3) = location.at(3);
    current_joint_group_positions.at(4) = location.at(4);
    current_joint_group_positions.at(5) = location.at(5);
    current_joint_group_positions.at(6) = location.at(6);

    

    kitting_robot_moveit_group.setJointValueTarget(current_joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan kitting_robot_path_plan;
    bool success = (kitting_robot_moveit_group.plan(kitting_robot_path_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        ROS_INFO("Robot is moving");
        kitting_robot_moveit_group.move();
    }
        
}

void KittingRobot::activate_gripper()
{
    nist_gear::VacuumGripperControl srv_msg;
    srv_msg.request.enable = true;
    kitting_robot_gripper_control_service_client.call(srv_msg);

    ROS_INFO_STREAM("Activate kitting robot gripper response : " << srv_msg.response);
}


void KittingRobot::deactivate_gripper()
{
    nist_gear::VacuumGripperControl srv_msg;
    srv_msg.request.enable = false;
    kitting_robot_gripper_control_service_client.call(srv_msg);

}

nist_gear::VacuumGripperState KittingRobot::get_gripper_state()
{
    return kitting_robot_gripper_state;
}


bool KittingRobot::pickup_part(geometry_msgs::Pose part_pose, double part_offset)
{
    activate_gripper();    

    // pose of the end effector in the world frame
    geometry_msgs::Pose arm_ee_link_pose = kitting_robot_moveit_group.getCurrentPose().pose;

    // pose to go to after grasping a part (lift the arm a little bit)
    geometry_msgs::Pose postGraspPose;
    geometry_msgs::Pose part_init_pose_in_world = part_pose;
    part_init_pose_in_world.position.z = part_init_pose_in_world.position.z + part_offset;
    part_init_pose_in_world.orientation.x = arm_ee_link_pose.orientation.x;
    part_init_pose_in_world.orientation.y = arm_ee_link_pose.orientation.y;
    part_init_pose_in_world.orientation.z = arm_ee_link_pose.orientation.z;
    part_init_pose_in_world.orientation.w = arm_ee_link_pose.orientation.w;

    // activate gripper
    ros::Duration(0.2).sleep();
    activate_gripper();
    auto state = get_gripper_state();


    if (!state.enabled) {
        int MAX_ATTEMPT = 10;
        for (int i{}; i < MAX_ATTEMPT; i++) {
            ros::Duration(0.2).sleep();
            activate_gripper();
            state = get_gripper_state();
        }
    }

    if (!state.enabled) {
        ROS_FATAL_STREAM("Kitting robot gripper state: Failed");
        ros::shutdown();
    }

    if (state.enabled) {
        ROS_INFO("Kitting robot gripper state: Enabled");

        //--Move arm to part
        kitting_robot_moveit_group.setPoseTarget(part_init_pose_in_world);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        kitting_robot_moveit_group.plan(plan);

        try
        {
            auto plan_result = kitting_robot_moveit_group.execute(plan);
        }
        catch (std::exception &ex) 
        {
            ROS_INFO("Kitting robot path plan not feasible!!");
            return false;
        }
        

        // move the arm closer until the object is attached
        state = get_gripper_state();        
        while (!state.attached) {

            try
            {
                part_init_pose_in_world.position.z = part_init_pose_in_world.position.z - 0.0005;
                kitting_robot_moveit_group.setPoseTarget(part_init_pose_in_world);
                kitting_robot_moveit_group.move();
                ros::Duration(0.5).sleep();
                kitting_robot_moveit_group.setPoseTarget(arm_ee_link_pose);                
                state = get_gripper_state();
            }
            catch (std::exception &ex)
            {
                ROS_INFO("Kitting robot path plan not feasible!!");
                return false;
            }

            
        }

        part_init_pose_in_world.position.z += 0.20;
        kitting_robot_moveit_group.setPoseTarget(part_init_pose_in_world);
        kitting_robot_moveit_group.move();


        ROS_INFO_STREAM("Part attached to the gripper");
        return true;
    }
    return false;

}


bool KittingRobot::place_part(geometry_msgs::Pose target_pose)
{
    geometry_msgs::Pose part_in_world_frame = target_pose;

    geometry_msgs::Pose target_pose_in_world;
    target_pose_in_world.position.x = part_in_world_frame.position.x;
    target_pose_in_world.position.y = part_in_world_frame.position.y;
    target_pose_in_world.position.z = part_in_world_frame.position.z;
    target_pose_in_world.orientation.x = part_in_world_frame.orientation.x;
    target_pose_in_world.orientation.y = part_in_world_frame.orientation.y;
    target_pose_in_world.orientation.z = part_in_world_frame.orientation.z;
    target_pose_in_world.orientation.w = part_in_world_frame.orientation.w;


    ROS_INFO("Target World Position: %f, %f, %f",
        part_in_world_frame.position.x,
        part_in_world_frame.position.y,
        part_in_world_frame.position.z);

    ROS_INFO("Target World Orientation: %f, %f, %f, %f",
        part_in_world_frame.orientation.x,
        part_in_world_frame.orientation.y,
        part_in_world_frame.orientation.z,
        part_in_world_frame.orientation.w);

    auto ee_pose = kitting_robot_moveit_group.getCurrentPose().pose;

    tf2::Quaternion q_current(
        ee_pose.orientation.x,
        ee_pose.orientation.y,
        ee_pose.orientation.z,
        ee_pose.orientation.w);

    
    // orientation of the part in the bin, in world frame
    tf2::Quaternion q_init_part(
        target_pose_in_world.orientation.x,
        target_pose_in_world.orientation.y,
        target_pose_in_world.orientation.z,
        target_pose_in_world.orientation.w);
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
    part_in_world_frame.position.z = 0.9;

    kitting_robot_moveit_group.setPoseTarget(part_in_world_frame);
    kitting_robot_moveit_group.move();
    ros::Duration(0.5).sleep();
    deactivate_gripper();
    
    auto state = get_gripper_state();
    if (state.attached) {
        return true;
    }
    
    else
        return false;
}


bool KittingRobot::kitting_robot_pick_place_service_callback(group2_rwa4::kitting_part_details::Request &req, group2_rwa4::kitting_part_details::Response &res)
{   
    go_to_location(arm_preset_at_home);

    std::vector<double> kitting_robot_initial_joint_pose;
    if(req.initial_preset_location == "home_conveyor")
    {
        kitting_robot_initial_joint_pose = arm_preset_at_home_conveyor;

    } else if(req.initial_preset_location == "bins0")
    {
        kitting_robot_initial_joint_pose = arm_preset_at_bins0;

    } else if(req.initial_preset_location == "bins1")
    {
        kitting_robot_initial_joint_pose = arm_preset_at_bins1;

    } else if(req.initial_preset_location == "agv1")
    {   
        kitting_robot_initial_joint_pose = arm_preset_at_agv1;

    } else if(req.initial_preset_location == "agv2")
    {
        kitting_robot_initial_joint_pose = arm_preset_at_agv2;

    } else if(req.initial_preset_location == "agv3")
    {
        kitting_robot_initial_joint_pose = arm_preset_at_agv3;

    } else if(req.initial_preset_location == "agv4")
    {
        kitting_robot_initial_joint_pose = arm_preset_at_agv4;

    } else if(req.initial_preset_location == "trash_bin")
    {
        kitting_robot_initial_joint_pose = arm_preset_at_trash_bin;
    }

    std::vector<double> kitting_robot_final_joint_pose;

    if(req.final_preset_location == "bins0")
    {
        kitting_robot_final_joint_pose = arm_preset_at_bins0;

    } else if(req.final_preset_location == "bins1")
    {
        kitting_robot_final_joint_pose = arm_preset_at_bins1;

    } else if(req.final_preset_location == "agv1")
    {   
        kitting_robot_final_joint_pose = arm_preset_at_agv1;

    } else if(req.final_preset_location == "agv2")
    {
        kitting_robot_final_joint_pose = arm_preset_at_agv2;

    } else if(req.final_preset_location == "agv3")
    {
        kitting_robot_final_joint_pose = arm_preset_at_agv3;

    } else if(req.final_preset_location == "agv4")
    {
        kitting_robot_final_joint_pose = arm_preset_at_agv4;

    } else if(req.final_preset_location == "trash_bin")
    {
        kitting_robot_final_joint_pose = arm_preset_at_trash_bin;

    }


    go_to_location(kitting_robot_initial_joint_pose);

    if (pickup_part(req.part_pose, req.part_offset)) 
    {
        // perform kitting
        go_to_location(kitting_robot_final_joint_pose);    
        
        if (place_part(req.part_target_pose)) 
        {
            go_to_location(kitting_robot_final_joint_pose);   
        }  
    } else 
    {
        go_to_location(kitting_robot_initial_joint_pose);
        return false;
    }

    return true;
    
}

bool KittingRobot::kitting_robot_dispose_faulty_service_callback(group2_rwa4::dispose_faulty_part::Request &req, group2_rwa4::dispose_faulty_part::Response &res)
{   
    ROS_INFO("Disposing faulty part");
    pickup_part(req.part_pose, req.part_offset);
    go_to_location(arm_preset_at_trash_bin);   
    deactivate_gripper();
    res.is_part_disposed = true;

    return true;
    
}




