#include "bins_handler.h"

BinsHandler::BinsHandler(ros::NodeHandle &node_handler)
{
    nh = node_handler;    
    init();
}

void BinsHandler::init()
{   
    logicam_bin0_sub = nh.subscribe("/ariac/logical_camera_1", 10, &BinsHandler::logicam_bins0_callback, this);
    logicam_bin1_sub = nh.subscribe("/ariac/logical_camera_2", 10, &BinsHandler::logicam_bins1_callback, this);
    check_part_service_bins0 = nh.advertiseService("/group2/check_part_bins0", &BinsHandler::check_part_service_bins0_callback, this);
    check_part_service_bins1 = nh.advertiseService("/group2/check_part_bins1", &BinsHandler::check_part_service_bins1_callback, this);
    check_part_pose_service_bins0 = nh.advertiseService("/group2/check_part_pose_bins0", &BinsHandler::check_part_pose_service_bins0_callback, this);
    check_part_pose_service_bins1 = nh.advertiseService("/group2/check_part_pose_bins1", &BinsHandler::check_part_pose_service_bins1_callback, this);

}

void BinsHandler::logicam_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    ROS_INFO_STREAM_ONCE("Logicam bin0 working");
    if(logicam_msg->models.size() > 0 && (bins0_parts.size()- logicam_msg->models.size()) > 0)
    {
        bins0_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            bins0_parts.push_back(logicam_msg->models[i]);
        }
        bins0_parts_count = bins0_parts.size();
        ROS_INFO_STREAM_ONCE("Bin 0 part count : " << bins0_parts_count);
    }
}



void BinsHandler::logicam_bins1_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    ROS_INFO_STREAM_ONCE("Logicam bin1 working");
    if(logicam_msg->models.size() > 0 && (bins1_parts.size() - logicam_msg->models.size()) > 0)
    {
        bins1_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            bins1_parts.push_back(logicam_msg->models[i]);
        }
        bins1_parts_count = bins1_parts.size();
        ROS_INFO_STREAM_ONCE("Bin 1 part count : " << bins1_parts_count);
    }

    nh.setParam("/is_bins_handler_initialized", true); // updating param server
}

bool BinsHandler::check_part_service_bins0_callback(group2_rwa4::check_exists::Request &req, group2_rwa4::check_exists::Response &res)
{       
    for(int i = 0; i < bins0_parts_count; i++)
    {        
        if(bins0_parts[i].type == req.part_type)
        {
            res.success = true;
            geometry_msgs::Pose world_frame_part_pose = transform_to_world_frame(bins0_parts[i].pose, "logical_camera_1_frame");
            res.part_poses.push_back(world_frame_part_pose);

            if(world_frame_part_pose.position.x < (-2.2753))
            {
                res.parts_present_in_bin0 = false;
                res.parts_present_in_bin1 = false;
                res.parts_present_in_bin2 = true;
                res.parts_present_in_bin3 = true;
                res.parts_present_in_bin4 = false;
                res.parts_present_in_bin5 = false;
                res.parts_present_in_bin6 = false;
                res.parts_present_in_bin7 = false;



            }
            else
            {
                res.parts_present_in_bin0 = true;
                res.parts_present_in_bin1 = true;
                res.parts_present_in_bin2 = false;
                res.parts_present_in_bin3 = false;
                res.parts_present_in_bin4 = false;
                res.parts_present_in_bin5 = false;
                res.parts_present_in_bin6 = false;
                res.parts_present_in_bin7 = false;
            }
        }
    }    
    return true;
}

bool BinsHandler::check_part_service_bins1_callback(group2_rwa4::check_exists::Request &req, group2_rwa4::check_exists::Response &res)
{       
    for(int i = 0; i < bins1_parts_count; i++)
    {        
        if(bins1_parts[i].type == req.part_type)
        {
            res.success = true;
            geometry_msgs::Pose world_frame_part_pose = transform_to_world_frame(bins1_parts[i].pose, "logical_camera_2_frame");
            res.part_poses.push_back(world_frame_part_pose);

            if(world_frame_part_pose.position.x < (-2.2753))
            {
                

                res.parts_present_in_bin0 = false;
                res.parts_present_in_bin1 = false;
                res.parts_present_in_bin2 = false;
                res.parts_present_in_bin3 = false;
                res.parts_present_in_bin4 = false;
                res.parts_present_in_bin5 = false;
                res.parts_present_in_bin6 = true;
                res.parts_present_in_bin7 = true;
                
            }
            else
            {
                res.parts_present_in_bin0 = false;
                res.parts_present_in_bin1 = false;
                res.parts_present_in_bin2 = false;
                res.parts_present_in_bin3 = false;
                res.parts_present_in_bin4 = true;
                res.parts_present_in_bin5 = true;
                res.parts_present_in_bin6 = false;
                res.parts_present_in_bin7 = false;
            }
        }
    }    
    return true;
}

bool BinsHandler::check_part_pose_service_bins0_callback(group2_rwa4::check_part_pose::Request &req, group2_rwa4::check_part_pose::Response &res)
{       
    for(int i = 0; i < bins0_parts_count; i++)
    {        
        if(bins0_parts[i].pose == req.part_pose)
        {
            ROS_INFO_STREAM(req.part_pose);
            res.success = true;
        }
    }    
    return true;
}

bool BinsHandler::check_part_pose_service_bins1_callback(group2_rwa4::check_part_pose::Request &req, group2_rwa4::check_part_pose::Response &res)
{       
    for(int i = 0; i < bins1_parts_count; i++)
    {        
        if(bins1_parts[i].pose == req.part_pose)
        {
            ROS_INFO_STREAM(req.part_pose);
            res.success = true;
        }
    }    
    return true;
}

geometry_msgs::Pose BinsHandler::transform_to_world_frame(const geometry_msgs::Pose& part_pose, std::string logicam_frame) 
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = logicam_frame;
    transformStamped.child_frame_id = "target_frame";
    transformStamped.transform.translation.x = part_pose.position.x;
    transformStamped.transform.translation.y = part_pose.position.y;
    transformStamped.transform.translation.z = part_pose.position.z;
    transformStamped.transform.rotation.x = part_pose.orientation.x;
    transformStamped.transform.rotation.y = part_pose.orientation.y;
    transformStamped.transform.rotation.z = part_pose.orientation.z;
    transformStamped.transform.rotation.w = part_pose.orientation.w;

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
            world_target_tf = tfBuffer.lookupTransform("world", "target_frame", ros::Time(0), timeout);
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