// #include "kitting_robot.h"
#include "gantry_robot.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf2/convert.h>


// This passes the parameters necessary for the planning group in the rviz gui
GantryRobot::GantryRobot(ros::NodeHandle& node) : nh("/ariac/gantry"),
    gantry_robot_planing_grp("/ariac/gantry/robot_description"),
    gantry_robot_moveit_options("gantry_full", gantry_robot_planing_grp, nh),
    gantry_robot_moveit_group(gantry_robot_moveit_options),
    gantry_robot_arm_moveit_options("gantry_arm", gantry_robot_planing_grp, nh),
    gantry_robot_arm_moveit_group(gantry_robot_arm_moveit_options)
{
    // nh = node;
    init();
}

void GantryRobot::init()
{
    ROS_INFO("Gantry Robot initialized :)");
    gantry_robot_joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);

    gantry_robot_arm_joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_arm_controller/command", 10);

    // joint state subscribers
    gantry_robot_joint_states_subscriber = nh.subscribe("/ariac/gantry/joint_states", 10, &GantryRobot::gantry_robot_joint_states_callback, this);

    // gripper state subscriber
    gantry_robot_gripper_state_subscriber = nh.subscribe("/ariac/gantry/arm/gripper/state", 10, &GantryRobot::gantry_robot_gripper_state_callback, this);
    
    // controller state subscribers
    gantry_robot_controller_state_subscriber = nh.subscribe("/ariac/gantry/gantry_controller/state", 10, &GantryRobot::gantry_robot_controller_state_callback, this);

    gantry_arm_controller_state_subscriber_ = nh.subscribe("/ariac/gantry/gantry_arm_controller/state", 10, &GantryRobot::gantry_arm_controller_state_callback, this);
    
    gantry_robot_gripper_control_service_client = nh.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/arm/gripper/control");
    gantry_robot_gripper_control_service_client.waitForExistence(); // wait till the service starts

    gantry_robot_pick_place_service = nh.advertiseService("/group2/gantry_robot_pick_and_place", &GantryRobot::gantry_robot_pick_place_service_callback, this);
    
    // go_to_location(arm_preset_at_center_1);

//################################################
///////////////////////////// Uncomment //////////////////////////////////////

    // ROS_INFO("Gantry Robot at Left side of Kitting Station");
    // go_to_location(arm_preset_at_bins_left);
    // ROS_INFO("Gantry Robot at bin 3");
    // go_to_location(arm_preset_at_bins2);
    // ROS_INFO("Gantry Robot at bin 4");
    // go_to_location(arm_preset_at_bins3);



    // ROS_INFO("Gantry Robot at Right side of Kitting Station");
    // go_to_location(arm_preset_at_bins_right);
    // ROS_INFO("Gantry Robot at bin 7");
    // go_to_location(arm_preset_at_bins6);
    // ROS_INFO("Gantry Robot at bin 8");
    // go_to_location(arm_preset_at_bins7);



    // ROS_ERROR("Gantry Robot at AGV 1");
    // go_to_location(arm_preset_at_agv1);
    // ROS_ERROR("Gantry Robot at AGV 2");
    // go_to_location(arm_preset_at_agv2);
    // ROS_ERROR("Gantry Robot at AGV 3");
    // go_to_location(arm_preset_at_agv3);
    // ROS_ERROR("Gantry Robot at AGV 4");
    // go_to_location(arm_preset_at_agv4);


    
    // ROS_INFO("Gantry Robot at ARIAC Center");
    // go_to_location(arm_preset_at_center_1);
    // ROS_INFO("Gantry Robot at Assembly Station 1");
    // go_to_location(arm_preset_at_as1);
    // ROS_INFO("Gantry Robot at AGV1 of AS1");
    // go_to_location(arm_preset_at_as1_agv1);
    // ROS_INFO("Gantry Robot at AGV2 of AS1");
    // go_to_location(arm_preset_at_as1_agv2);


    
    // ROS_INFO("Gantry Robot at ARIAC Center");
    // go_to_location(arm_preset_at_center_1);
    // ROS_INFO("Gantry Robot at ARIAC Center");
    // go_to_location(arm_preset_at_center_2);
    // ROS_INFO("Gantry Robot at Assembly Station 2");
    // go_to_location(arm_preset_at_as2);
    // ROS_INFO("Gantry Robot at AGV1 of AS2");
    // go_to_location(arm_preset_at_as2_agv1);
    // ROS_INFO("Gantry Robot at AGV2 of AS2");
    // go_to_location(arm_preset_at_as2_agv2);


    
    // ROS_INFO("Gantry Robot at ARIAC Center");
    // go_to_location(arm_preset_at_center_2);
    // ROS_INFO("Gantry Robot at ARIAC Center");
    // go_to_location(arm_preset_at_center_1);
    // ROS_INFO("Gantry Robot at Assembly Station 3");
    // go_to_location(arm_preset_at_as3);
    // ROS_INFO("Gantry Robot at AGV3 of AS3");
    // go_to_location(arm_preset_at_as3_agv3);
    // ROS_INFO("Gantry Robot at AGV4 of AS3");
    // go_to_location(arm_preset_at_as3_agv4);


    // ROS_INFO("Gantry Robot at ARIAC Center");
    // go_to_location(arm_preset_at_center_1);
    // ROS_INFO("Gantry Robot at ARIAC Center");
    // go_to_location(arm_preset_at_center_2);
    // ROS_INFO("Gantry Robot at Assembly Station 4");
    // go_to_location(arm_preset_at_as4);
    // ROS_INFO("Gantry Robot at AGV3 of AS4");
    // go_to_location(arm_preset_at_as4_agv3);
    // ROS_INFO("Gantry Robot at AGV4 of AS4");
    // go_to_location(arm_preset_at_as4_agv4);



    // ROS_INFO("Gantry Robot at ARIAC Center");
    // go_to_location(arm_preset_at_center_2);
    // ROS_INFO("Gantry Robot at ARIAC Center");
    // go_to_location(arm_preset_at_center_1);
    
//################################################
///////////////////////////// Uncomment //////////////////////////////////////

    // gantry_robot_dispose_faulty_service = nh.advertiseService("/group2/gantry_dispose_faulty", &GantryRobot::gantry_robot_dispose_faulty_service_callback, this);


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

  
    

    ///////////////////////////////////////////////////////////////
    // safebins.gantry_torso_preset = { -6.90, -0.13, -0.02 };
    // safebins.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
    // safebins.gantry_full_preset.insert(safebins.gantry_full_preset.begin(), safebins.gantry_torso_preset.begin(), safebins.gantry_torso_preset.end());
    // safebins.gantry_full_preset.insert(safebins.gantry_full_preset.end(), safebins.gantry_arm_preset.begin(), safebins.gantry_arm_preset.end());
    // home_.gantry_torso_preset = { -3.90, -0.13, -0.02 };
    // home_.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
    // //concatenate gantry torso and gantry arm
    // home_.gantry_full_preset.insert(home_.gantry_full_preset.begin(), home_.gantry_torso_preset.begin(), home_.gantry_torso_preset.end());
    // home_.gantry_full_preset.insert(home_.gantry_full_preset.end(), home_.gantry_arm_preset.begin(), home_.gantry_arm_preset.end());
    // // print(home_.gantry_full);


    // safebins.gantry_torso_preset = { -0.3, -2.0, 0.0  };
    // safebins.gantry_arm_preset = { 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
    // safebins.gantry_full_preset.insert(safebins.gantry_full_preset.begin(), safebins.gantry_torso_preset.begin(), safebins.gantry_torso_preset.end());
    // safebins.gantry_full_preset.insert(safebins.gantry_full_preset.end(), safebins.gantry_arm_preset.begin(), safebins.gantry_arm_preset.end());

    const moveit::core::JointModelGroup* gantry_robot_joint_model_group = gantry_robot_moveit_group.getCurrentState()->getJointModelGroup("gantry_full");
    moveit::core::RobotStatePtr gantry_robot_current_state = gantry_robot_moveit_group.getCurrentState();
    gantry_robot_current_state->copyJointGroupPositions(gantry_robot_joint_model_group, current_joint_group_positions);


    const moveit::core::JointModelGroup* joint_arm_group = gantry_robot_arm_moveit_group.getCurrentState()->getJointModelGroup("gantry_arm");
    moveit::core::RobotStatePtr current_state_arm = gantry_robot_arm_moveit_group.getCurrentState();
    current_state_arm->copyJointGroupPositions(joint_arm_group, joint_arm_positions_);

}

void GantryRobot::gantry_robot_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& vacuum_state_msg)
{   
    gantry_robot_gripper_state = *vacuum_state_msg;
}

void GantryRobot::gantry_robot_joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
{
    if (joint_state_msg->position.size() == 0) {
        ROS_ERROR("[Arm][arm_joint_states_callback_] joint_state_msg->position.size() == 0!");
    }
    current_gantry_robot_joint_states = *joint_state_msg;
}

void GantryRobot::gantry_robot_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& controller_state_msg)
{
    gantry_robot_controller_state = *controller_state_msg;
}

void GantryRobot::gantry_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    gantry_arm_controller_state_ = *msg;
}

// void GantryRobot::go_to_location(std::vector<double> location)
// {

//     current_joint_group_positions.clear();
//     // current_joint_group_positions.at(0) = location.at(0);
//     // current_joint_group_positions.at(1) = location.at(1);
//     // current_joint_group_positions.at(2) = location.at(2);
//     // current_joint_group_positions.at(3) = location.at(3);
//     // current_joint_group_positions.at(4) = location.at(4);
//     // current_joint_group_positions.at(5) = location.at(5);
//     // current_joint_group_positions.at(6) = location.at(6);
//     for (int i = 0; i < location.size(); i++)
//     {
//         current_joint_group_positions.push_back(location.at(i));
//         ROS_ERROR_STREAM("go to gantry :"<< location.at(i) );
//     }
    

//     gantry_robot_moveit_group.setJointValueTarget(current_joint_group_positions);

//     moveit::planning_interface::MoveGroupInterface::Plan gantry_robot_path_plan;
//     bool success = (gantry_robot_moveit_group.plan(gantry_robot_path_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if (success)
//     {
//         ROS_INFO("Gantry Robot is moving");
//         gantry_robot_moveit_group.move();
//     }
        
// }

void GantryRobot::go_to_location(std::vector<double> location)
{

    // current_joint_group_positions.at(0) = location.gantry_torso_preset.at(0);
    // current_joint_group_positions.at(1) = location.gantry_torso_preset.at(1);
    // current_joint_group_positions.at(2) = location.gantry_torso_preset.at(2);
    // // gantry arm
    // current_joint_group_positions.at(3) = location.gantry_arm_preset.at(0);
    // current_joint_group_positions.at(4) = location.gantry_arm_preset.at(1);
    // current_joint_group_positions.at(5) = location.gantry_arm_preset.at(2);
    // current_joint_group_positions.at(6) = location.gantry_arm_preset.at(3);
    // current_joint_group_positions.at(7) = location.gantry_arm_preset.at(4);
    // current_joint_group_positions.at(8) = location.gantry_arm_preset.at(5);


    current_joint_group_positions.clear();
    // current_joint_group_positions.at(0) = location.at(0);
    // current_joint_group_positions.at(1) = location.at(1);
    // current_joint_group_positions.at(2) = location.at(2);
    // current_joint_group_positions.at(3) = location.at(3);
    // current_joint_group_positions.at(4) = location.at(4);
    // current_joint_group_positions.at(5) = location.at(5);
    // current_joint_group_positions.at(6) = location.at(6);
    for (int i = 0; i < location.size(); i++)
    {
        current_joint_group_positions.push_back(location.at(i));
        ROS_ERROR_STREAM("go to gantry :"<< location.at(i) );
    }


    gantry_robot_moveit_group.setJointValueTarget(current_joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan gantry_robot_path_plan;
    bool success = (gantry_robot_moveit_group.plan(gantry_robot_path_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        ROS_INFO("Gantry Robot is moving");
        gantry_robot_moveit_group.move();
    }

}

void GantryRobot::activate_gripper()
{
    nist_gear::VacuumGripperControl srv_msg;
    srv_msg.request.enable = true;
    gantry_robot_gripper_control_service_client.call(srv_msg);

    ROS_INFO_STREAM("Activate gantry robot gripper response : " << srv_msg.response);
}


void GantryRobot::deactivate_gripper()
{
    nist_gear::VacuumGripperControl srv_msg;
    srv_msg.request.enable = false;
    gantry_robot_gripper_control_service_client.call(srv_msg);

}

nist_gear::VacuumGripperState GantryRobot::get_gripper_state()
{
    return gantry_robot_gripper_state;
}


bool GantryRobot::pickup_part(geometry_msgs::Pose part_pose, double part_offset)
{
    // const double GRIPPER_HEIGHT = 0.01;
    // const double EPSILON = 0.008; // for the gripper to firmly touch
    // ros::Duration(3).sleep();
    activate_gripper();
    const double GRIPPER_HEIGHT = 0.01;
    const double EPSILON = 0.008; // for the gripper to firmly touch
    // ros::Duration(0.5).sleep();

        

    geometry_msgs::Pose gantry_ee_link_pose = gantry_robot_arm_moveit_group.getCurrentPose().pose;
    tf2::Quaternion  gantry_ee_link_orientation(
        gantry_ee_link_pose.orientation.x,
        gantry_ee_link_pose.orientation.y,
        gantry_ee_link_pose.orientation.z,
        gantry_ee_link_pose.orientation.w
    );

    // // pose of the end effector in the world frame
    // geometry_msgs::Pose arm_ee_link_pose = gantry_robot_moveit_group.getCurrentPose().pose;

    // pose to go to after grasping a part (lift the arm a little bit)
    geometry_msgs::Pose postGraspPose;
    geometry_msgs::Pose part_init_pose_in_world = part_pose;
    part_init_pose_in_world.position.z = part_init_pose_in_world.position.z + part_offset; // + GRIPPER_HEIGHT - EPSILON;
    part_init_pose_in_world.orientation.x = gantry_ee_link_pose.orientation.x;
    part_init_pose_in_world.orientation.y = gantry_ee_link_pose.orientation.y;
    part_init_pose_in_world.orientation.z = gantry_ee_link_pose.orientation.z;
    part_init_pose_in_world.orientation.w = gantry_ee_link_pose.orientation.w;

    // activate gripper
    ros::Duration(3).sleep();
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
        ROS_FATAL_STREAM("Gantry robot gripper state: Failed");
        ros::shutdown();
    }

    if (state.enabled) {
        ROS_INFO("Gantry robot gripper state: Enabled");

        //--Move arm to part
        gantry_robot_moveit_group.setPoseTarget(part_init_pose_in_world);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        gantry_robot_moveit_group.plan(plan);

        try
        {
            auto plan_result = gantry_robot_moveit_group.execute(plan);
        }
        catch (std::exception &ex) 
        {
            ROS_INFO("Gantry robot path plan not feasible!!");
            return false;
        }
        

        // move the arm closer until the object is attached
        state = get_gripper_state();        
        while (!state.attached) {

            try
            {
                part_init_pose_in_world.position.z = part_init_pose_in_world.position.z - 0.0005;
                gantry_robot_moveit_group.setPoseTarget(part_init_pose_in_world);
                gantry_robot_moveit_group.move();
                ros::Duration(0.1).sleep();
                gantry_robot_moveit_group.setPoseTarget(gantry_ee_link_pose);                
                state = get_gripper_state();
            }
            catch (std::exception &ex)
            {
                ROS_INFO("Gantry robot path plan not feasible!!");
                return false;
            }

            
        }

        part_init_pose_in_world.position.z += 0.20;
        gantry_robot_moveit_group.setPoseTarget(part_init_pose_in_world);
        gantry_robot_moveit_group.move();


        ROS_INFO_STREAM("Part attached to the gripper");
        return true;
    }
    return false;

}


bool GantryRobot::place_part(geometry_msgs::Pose target_pose)
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

    auto ee_pose = gantry_robot_moveit_group.getCurrentPose().pose;

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

    gantry_robot_moveit_group.setPoseTarget(part_in_world_frame);
    gantry_robot_moveit_group.move();
    ros::Duration(0.5).sleep();
    deactivate_gripper();
    
    auto state = get_gripper_state();
    if (state.attached) {
        return true;
    }
    
    else
        return false;
}


bool GantryRobot::gantry_robot_pick_place_service_callback(group2_rwa4::kitting_part_details::Request &req, group2_rwa4::kitting_part_details::Response &res)
{   
    std::vector<double> gantry_robot_initial_joint_pose;
    // go_to_location(arm_preset_at_center_1);

    if(req.initial_preset_location == "bins0")
    {
        // gantry_robot_initial_joint_pose = safebins;
        go_to_location(arm_preset_at_bins_left);

    }

    // if(req.initial_preset_location == "bins0")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_bins0;

    // } else if(req.initial_preset_location == "bins1")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_bins1;

    // } else if(req.initial_preset_location == "agv1")
    // {   
    //     gantry_robot_initial_joint_pose = arm_preset_at_agv1;

    // } else if(req.initial_preset_location == "agv2")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_agv2;

    // } else if(req.initial_preset_location == "agv3")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_agv3;

    // } else if(req.initial_preset_location == "agv4")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_agv4;

    // }

    ////////////////////////////////////////
    // INITIAL PRESET Location
    //////////////////////////////////////////////////////////////////////////
    // if(req.initial_preset_location == "bins0")// "bins_right")
    // {
    //     // go_to_location(arm_preset_at_bins3);
    //     go_to_location(safebins);
        

    // } else if(req.initial_preset_location == "bins1")// "bins_right")
    // {
    //     go_to_location(arm_preset_at_bins_right);
    // } 
    // else if(req.initial_preset_location == "bins2")// "bins_left")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_bins_left;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_bins2;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "bins3")// "bins_right")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_bins_left;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_bins3;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "bins6")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_bins_right;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_bins6;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "bins7")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_bins_right;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_bins7;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // }
    // else if(req.initial_preset_location == "agv1")
    // {   
    //     gantry_robot_initial_joint_pose = arm_preset_at_agv1;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "agv2")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_agv2;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "agv3")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_agv3;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "agv4")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_agv4;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // }
    // else if(req.initial_preset_location == "as1_agv1")// "bins_left")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_pre_as1;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_as1_agv1;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "as1_agv2")// "bins_right")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_pre_as1;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_as1_agv2;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "as2_agv1")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_pre_as2;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_as2_agv1;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "as2_agv2")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_pre_as2;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_as2_agv2;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // }
    // else if(req.initial_preset_location == "as3_agv3")
    // {   
    //     gantry_robot_initial_joint_pose = arm_preset_at_pre_as3;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_as3_agv3;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "as3_agv4")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_pre_as3;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_as3_agv4;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "as4_agv3")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_pre_as4;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_as4_agv3;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "as4_agv4")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_pre_as4;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_as4_agv4;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // }
    // else if(req.initial_preset_location == "as1")
    // {   
    //     gantry_robot_initial_joint_pose = arm_preset_at_pre_as1;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_as1;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "as2")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_pre_as2;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_as2;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "as3")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_pre_as3;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_as3;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // } 
    // else if(req.initial_preset_location == "as4")
    // {
    //     gantry_robot_initial_joint_pose = arm_preset_at_pre_as4;
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     gantry_robot_initial_joint_pose = arm_preset_at_as4;
    //     go_to_location(gantry_robot_initial_joint_pose);

    // }

    // std::vector<double> gantry_robot_final_joint_pose;

    // // if(req.final_preset_location == "bins0")
    // // {
    // //     gantry_robot_final_joint_pose = arm_preset_at_bins0;

    // // } else if(req.final_preset_location == "bins1")
    // // {
    // //     gantry_robot_final_joint_pose = arm_preset_at_bins1;

    // // } else if(req.final_preset_location == "agv1")
    // // {   
    // //     gantry_robot_final_joint_pose = arm_preset_at_agv1;

    // // } else if(req.final_preset_location == "agv2")
    // // {
    // //     gantry_robot_final_joint_pose = arm_preset_at_agv2;

    // // } else if(req.final_preset_location == "agv3")
    // // {
    // //     gantry_robot_final_joint_pose = arm_preset_at_agv3;

    // // } else if(req.final_preset_location == "agv4")
    // // {
    // //     gantry_robot_final_joint_pose = arm_preset_at_agv4;

    // // }

    // ////////////////////////////////////////
    // // FINAL PRESET Location
    // //////////////////////////////////////////////////////////////////////////
    // if(req.final_preset_location == "bins2")// "bins_left")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_bins_left;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_bins2;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "bins3")// "bins_right")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_bins_left;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_bins3;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "bins6")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_bins_right;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_bins6;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "bins7")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_bins_right;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_bins7;
    //     go_to_location(gantry_robot_final_joint_pose);

    // }
    // else if(req.final_preset_location == "agv1")
    // {   
    //     gantry_robot_final_joint_pose = arm_preset_at_agv1;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "agv2")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_agv2;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "agv3")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_agv3;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "agv4")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_agv4;
    //     // go_to_location(gantry_robot_final_joint_pose);

    // }
    // else if(req.final_preset_location == "as1_agv1")// "bins_left")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_pre_as1;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_as1_agv1;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "as1_agv2")// "bins_right")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_pre_as1;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_as1_agv2;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "as2_agv1")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_pre_as2;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_as2_agv1;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "as2_agv2")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_pre_as2;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_as2_agv2;
    //     go_to_location(gantry_robot_final_joint_pose);

    // }
    // else if(req.final_preset_location == "as3_agv3")
    // {   
    //     gantry_robot_final_joint_pose = arm_preset_at_pre_as3;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_as3_agv3;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "as3_agv4")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_pre_as3;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_as3_agv4;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "as4_agv3")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_pre_as4;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_as4_agv3;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "as4_agv4")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_pre_as4;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_as4_agv4;
    //     go_to_location(gantry_robot_final_joint_pose);

    // }
    // else if(req.final_preset_location == "as1")
    // {   
    //     gantry_robot_final_joint_pose = arm_preset_at_pre_as1;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_as1;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "as2")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_pre_as2;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_as2;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "as3")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_pre_as3;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_as3;
    //     go_to_location(gantry_robot_final_joint_pose);

    // } 
    // else if(req.final_preset_location == "as4")
    // {
    //     gantry_robot_final_joint_pose = arm_preset_at_pre_as4;
    //     go_to_location(gantry_robot_final_joint_pose);
    //     gantry_robot_final_joint_pose = arm_preset_at_as4;
    //     go_to_location(gantry_robot_final_joint_pose);

    // }


    // go_to_location(gantry_robot_initial_joint_pose);

    // if (pickup_part(req.part_pose, req.part_offset)) 
    // {
    //     // perform kitting
    //     go_to_location(gantry_robot_final_joint_pose);    
        
    //     if (place_part(req.part_target_pose)) 
    //     {
    //         go_to_location(gantry_robot_final_joint_pose);   
    //     }  
    // } else 
    // {
    //     go_to_location(gantry_robot_initial_joint_pose);
    //     return false;
    // }

////////////////////////////////////////////////////////////////
    
    pickup_part(req.part_pose, req.part_offset);
    // if(!pickup_part(req.part_pose, req.part_offset))
    // {
    //     ROS_ERROR("Pick up part Failed");
    // }
    // if(!place_part(req.part_target_pose)
    // {
    //     ROS_ERROR("Place part Failed");
    // }
    ros::Duration(1).sleep();
    go_to_location(arm_preset_at_agv4);
    place_part(req.part_target_pose);


    return true;
    
}

// bool GantryRobot::gantry_robot_dispose_faulty_service_callback(group2_rwa4::dispose_faulty_part::Request &req, group2_rwa4::dispose_faulty_part::Response &res)
// {   
//     ROS_INFO("Disposing faulty part");
//     pickup_part(req.part_pose, req.part_offset);
//     go_to_location(arm_preset_at_trash_bin);   
//     deactivate_gripper();
//     res.is_part_disposed = true;

//     return true;
    
// }




