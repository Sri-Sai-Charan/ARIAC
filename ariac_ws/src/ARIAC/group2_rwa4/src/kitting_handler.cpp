#include <ros/ros.h>
#include "kitting_handler.h"
#include "order_handler.h"
#include <string.h>

#include <group2_rwa4/list_all_parts.h>
#include "sensor_array.h"


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
KittingHandler::KittingHandler(ros::NodeHandle& node) 
{
    nh = node;
    init();    
}

void KittingHandler::init()
{
    ROS_INFO("KittingHandler initialized :)");
    order_kitting_shipment_client = nh.serviceClient<group2_rwa4::order_kitting_shipment_details>("/group2/get_order_kitting_shipment_details");
    sensor_bins0_client = nh.serviceClient<group2_rwa4::check_exists>("/group2/check_part_bins0");
    sensor_bins1_client = nh.serviceClient<group2_rwa4::check_exists>("/group2/check_part_bins1");
    kitting_robot_pick_place_client = nh.serviceClient<group2_rwa4::kitting_part_details>("/group2/kitting_robot_pick_and_place"); 
    order_completion_status_client = nh.serviceClient<group2_rwa4::order_completion_status>("/group2/order_completion_status");   

    kitting_task_sub = nh.subscribe("/group2/kitting_task", 1, &KittingHandler::kitting_task_sub_callback, this);
    task_completed_srv_client = nh.serviceClient<std_srvs::Trigger>("/group2/task_completed");
    dispose_faulty_srv_client = nh.serviceClient<std_srvs::Trigger>("/group2/dispose_faulty");
    gantry_robot_pick_place_client = nh.serviceClient<group2_rwa4::kitting_part_details>("/group2/gantry_robot_pick_and_place");

    // sensor_bins0_client = nh.serviceClient<group2_rwa4::check_exists>("/group2/check_part_bins0");
    // sensor_bins1_client = nh.serviceClient<group2_rwa4::check_exists>("/group2/check_part_bins1");
    
}

void KittingHandler::kitting_task_sub_callback(group2_rwa4::Task kitting_task)
{
    // ROS_INFO_STREAM(kitting_task);
    perform_part_kitting(kitting_task);
    
}




void KittingHandler::perform_part_kitting(group2_rwa4::Task kitting_task)
{     
    ROS_INFO("Performing part kitting");
    nh.setParam("/is_kitting_robot_busy", true);
    if(kitting_task.task_type == "kitting")
    {
        group2_rwa4::kitting_part_details kitting_part_details_srv_msg;
        kitting_part_details_srv_msg.request.initial_preset_location = kitting_task.initial_preset_location;
        kitting_part_details_srv_msg.request.final_preset_location = kitting_task.final_preset_location;
        kitting_part_details_srv_msg.request.part_pose = kitting_task.part_pose;
        kitting_part_details_srv_msg.request.part_target_pose = kitting_task.part_target_pose;
        
        ///////////////////////////////////////////////Sai//////////////////////////////////////////////////////////////////////
        // group2_rwa4::check_exists bins0_srv_msg;
        // group2_rwa4::check_exists bins1_srv_msg;
        

        // bins0_srv_msg.request.part_type = kitting_task.part_type;
        // sensor_bins0_client.call(bins0_srv_msg);

        // bins1_srv_msg.request.part_type = kitting_task.part_type;
        // sensor_bins1_client.call(bins1_srv_msg);
        ///////////////////////////////////////////////Sai//////////////////////////////////////////////////////////////////////
        // spliting the part type string
        std::string part_type = kitting_task.part_type;
        std::string delimiter = "_";
        std::vector<std::string> split_words{};

        std::size_t pos;
        while ((pos = part_type.find(delimiter)) != std::string::npos) {
            split_words.push_back(part_type.substr(0, pos));
            part_type.erase(0, pos + delimiter.length());
        }

        std::string part_name = split_words[1];

        // setting part offset pose value
        if(part_name == "battery")
        {
            kitting_part_details_srv_msg.request.part_offset = 0.045;
        } 
        else if(part_name == "sensor")
        {
            kitting_part_details_srv_msg.request.part_offset = 0.053;
        }
        else if(part_name == "regulator")
        {
            kitting_part_details_srv_msg.request.part_offset = 0.06;
        }
        else
        {
            kitting_part_details_srv_msg.request.part_offset = 0.08;
        } 

        ROS_INFO_STREAM("Moving to part offset("<< kitting_part_details_srv_msg.request.part_offset<<") location for pickup");
    
        ///////////////////////////////////////////////Sai//////////////////////////////////////////////////////////////////////
        // if(kitting_task.part_pose.position.x > (-2.2753))
        //     {
        //         if(!kitting_robot_pick_place_client.call(kitting_part_details_srv_msg))
        //         {
        //             ROS_INFO("Retrying part pick and place");
        //             kitting_robot_pick_place_client.call(kitting_part_details_srv_msg);
        //         }
        //     }

        // else 
        // if(kitting_task.part_pose.position.x < (-2.2753))

        // {
        //     ROS_INFO_STREAM(kitting_part_details_srv_msg.request);
        //     if(!gantry_robot_pick_place_client.call(kitting_part_details_srv_msg))
        //     {
        //         ROS_INFO("Retrying part pick and place");
        //         gantry_robot_pick_place_client.call(kitting_part_details_srv_msg);
        //     }
        // }
        ///////////////////////////////////////////////Sai//////////////////////////////////////////////////////////////////////
        group2_rwa4::check_agv_faulty_parts agv_faulty_part_srv_msg;

        // checking for faulty part on the agv
        std::string agv_id = kitting_task.final_preset_location;
        ros::ServiceClient agv_qc_client = nh.serviceClient<group2_rwa4::check_agv_faulty_parts>("/group2/check_"+agv_id+"_faulty_parts");

        agv_faulty_part_srv_msg.request.agv_id = agv_id;
        agv_qc_client.call(agv_faulty_part_srv_msg);

        ROS_INFO("Checking for faulty part");
        ROS_INFO_STREAM(agv_faulty_part_srv_msg.response.faulty_part_poses.size());

        // delay to add task having higher priority
        nh.setParam("/is_kitting_robot_busy", false);
        ros::Duration(2).sleep();

        if(agv_faulty_part_srv_msg.response.faulty_part_poses.size() > 0)
        {   
            std_srvs::Trigger dispose_faulty_srv_msg;
            dispose_faulty_srv_client.call(dispose_faulty_srv_msg);
        }
        else
        {
            std_srvs::Trigger task_completed_srv_msg;
            task_completed_srv_client.call(task_completed_srv_msg);
        }
        
    }

    if(kitting_task.task_type == "dispose_faulty")
    {    

        group2_rwa4::dispose_faulty_part faulty_part_pose;
        faulty_part_pose.request.part_pose = kitting_task.part_target_pose;

        // spliting the part type string
        std::string part_type = kitting_task.part_type;
        std::string delimiter = "_";
        std::vector<std::string> split_words{};

        std::size_t pos;
        while ((pos = part_type.find(delimiter)) != std::string::npos) {
            split_words.push_back(part_type.substr(0, pos));
            part_type.erase(0, pos + delimiter.length());
        }

        std::string part_name = split_words[1];

        // setting part offset pose value
        if(part_name == "battery")
        {
            faulty_part_pose.request.part_offset = 0.045;
        } 
        else if(part_name == "sensor")
        {
            faulty_part_pose.request.part_offset = 0.09;
        }
        else if(part_name == "regulator")
        {
            faulty_part_pose.request.part_offset = 0.06;
        }
        else
        {
            faulty_part_pose.request.part_offset = 0.08;
        } 

        ROS_INFO_STREAM("Moving to part offset("<< faulty_part_pose.request.part_offset<<") location for pickup");
    
        ros::ServiceClient kitting_robot_dispose_faulty_client = nh.serviceClient<group2_rwa4::dispose_faulty_part>("/group2/kitting_dispose_faulty");
        
        ROS_INFO("Disposing faulty part");
        kitting_robot_dispose_faulty_client.call(faulty_part_pose);

        // delay to add task having higher priority
        nh.setParam("/is_kitting_robot_busy", false);
        ros::Duration(2).sleep();

        std_srvs::Trigger task_completed_srv_msg;
        task_completed_srv_client.call(task_completed_srv_msg);
    }



}

void KittingHandler::submit_agv(std::string agv_id, std::string assembly_station, std::string shipment_type)
{
    // sumbitting agv
    ros::ServiceClient shipment_submit_client = nh.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/"+agv_id+"/submit_shipment");
    if (!shipment_submit_client.exists())
    {
        shipment_submit_client.waitForExistence();
    }

    ROS_INFO("Requesting the service...");
    nist_gear::AGVToAssemblyStation srv_msg;
    srv_msg.request.assembly_station_name = assembly_station;
    srv_msg.request.shipment_type = shipment_type;

    shipment_submit_client.call(srv_msg);

    if (!srv_msg.response.success)
    {
        ROS_ERROR_STREAM("Failed to submit agv"<< agv_id <<" to assembly station " << srv_msg.request.assembly_station_name);
    }
    else
    {
        ROS_INFO_STREAM(agv_id << " moving to: " << srv_msg.request.assembly_station_name);
    }
}

geometry_msgs::Pose KittingHandler::get_part_target_pose(const geometry_msgs::Pose& target_pose, std::string target_frame, std::string child_frame_id) 
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = target_frame;
    transformStamped.child_frame_id = child_frame_id;
    transformStamped.transform.translation.x = target_pose.position.x;
    transformStamped.transform.translation.y = target_pose.position.y;
    transformStamped.transform.translation.z = target_pose.position.z;
    transformStamped.transform.rotation.x = target_pose.orientation.x;
    transformStamped.transform.rotation.y = target_pose.orientation.y;
    transformStamped.transform.rotation.z = target_pose.orientation.z;
    transformStamped.transform.rotation.w = target_pose.orientation.w;

    // broadcasting transformed frame
    for (int i{ 0 }; i < 15; ++i)
        br.sendTransform(transformStamped);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);
    ros::Duration timeout(1.0);

    geometry_msgs::TransformStamped world_target_tf;

    for (int i = 0; i < 10; i++) {
        try {
            world_target_tf = tfBuffer.lookupTransform("world", child_frame_id, ros::Time(0), timeout);
        }
        catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    geometry_msgs::Pose world_target{};

    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = world_target_tf.transform.rotation.x;
    world_target.orientation.y = world_target_tf.transform.rotation.y;
    world_target.orientation.z = world_target_tf.transform.rotation.z;
    world_target.orientation.w = world_target_tf.transform.rotation.w;

    return world_target;
}

