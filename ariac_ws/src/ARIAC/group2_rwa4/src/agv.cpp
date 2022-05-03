#include "agv.h"

AGV::AGV(ros::NodeHandle &node_handler, int id)
{
    nh = node_handler;    
    agv_id = std::to_string(id);
    agv_name = "agv"+ std::to_string(id);

    init();
}

void AGV::init()
{   
    ROS_INFO_STREAM(agv_name<< " is initialized :)" );
    quality_sensor_sub = nh.subscribe("/ariac/quality_control_sensor_"+agv_id, 10, &AGV::check_faulty_part, this);
    check_agv_faulty_part_service = nh.advertiseService("/group2/check_agv"+agv_id+"_faulty_parts", &AGV::check_agv_faulty_part_service_callback, this);

    // agv_details_at_as_service = nh.advertiseService("/group2/agv_details_at_as"+agv_id, &AGV::agv_details_at_as_service_callback, this);

    agv_logicam_sub = nh.subscribe("/ariac/logical_camera_agv"+agv_id, 10, &AGV::agv_logicam_callback, this);

    /////////////////////////////////////////////// JO ///////////////////////////////////////
    // logical_camera_as1_sub = nh.subscribe("/ariac/logical_camera_as1", 10, &AGV::logical_camera_as1_callback, this);
    // logical_camera_as2_sub = nh.subscribe("/ariac/logical_camera_as2", 10, &AGV::logical_camera_as2_callback, this);
    // logical_camera_as3_sub = nh.subscribe("/ariac/logical_camera_as3", 10, &AGV::logical_camera_as3_callback, this);
    // logical_camera_as4_sub = nh.subscribe("/ariac/logical_camera_as4", 10, &AGV::logical_camera_as4_callback, this);

    // logical_camera_as1_agv1_sub = nh.subscribe("/ariac/logical_camera_as1_agv1", 10, &AGV::logical_camera_as1_agv1_callback, this);
    // logical_camera_as1_agv2_sub = nh.subscribe("/ariac/logical_camera_as1_agv2", 10, &AGV::logical_camera_as1_agv2_callback, this);
    // logical_camera_as2_agv1_sub = nh.subscribe("/ariac/logical_camera_as2_agv1", 10, &AGV::logical_camera_as2_agv1_callback, this);
    // logical_camera_as2_agv2_sub = nh.subscribe("/ariac/logical_camera_as2_agv2", 10, &AGV::logical_camera_as2_agv2_callback, this);
    // logical_camera_as3_agv3_sub = nh.subscribe("/ariac/logical_camera_as3_agv3", 10, &AGV::logical_camera_as3_agv3_callback, this);
    // logical_camera_as3_agv4_sub = nh.subscribe("/ariac/logical_camera_as3_agv4", 10, &AGV::logical_camera_as3_agv4_callback, this);
    // logical_camera_as4_agv3_sub = nh.subscribe("/ariac/logical_camera_as4_agv3", 10, &AGV::logical_camera_as4_agv3_callback, this);
    // logical_camera_as4_agv4_sub = nh.subscribe("/ariac/logical_camera_as4_agv4", 10, &AGV::logical_camera_as4_agv4_callback, this);


}


void AGV::check_faulty_part(const nist_gear::LogicalCameraImage::ConstPtr &quality_sensor_msg)
{   
    if(agv_station == "ks"+agv_id)
    {
        if(quality_sensor_msg->models.size() > 0)
        {
            ROS_INFO_STREAM_ONCE("Faulty Part detected in "<< agv_name);
            faulty_part_count = quality_sensor_msg->models.size();
            faulty_part_poses.clear();
            for(int i=0; i< quality_sensor_msg->models.size(); i++)
            {

                faulty_part_poses.push_back(quality_sensor_msg->models[i].pose);                
                
            }
        }
        else
        {
            faulty_part_count = 0;
        }
    }   
}


bool AGV::check_agv_faulty_part_service_callback(group2_rwa4::check_agv_faulty_parts::Request &req, group2_rwa4::check_agv_faulty_parts::Response &res)
{       
          
    if(agv_name == req.agv_id)
    {
        res.success = true;
        if(faulty_part_count > 0)
        {
            res.faulty_part_count = std::to_string(faulty_part_count);
            for(int i=0; i< faulty_part_poses.size(); i++)
            {   
                res.faulty_part_poses.push_back(faulty_part_poses[i]);                
            }
        }
        else
        {
            res.faulty_part_count = std::to_string(0);
        }
        
    }
       
    return true;
}




void AGV::agv_logicam_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{    
}

// void AGV::submit_agv(std::string assembly_station, std::string shipment_type)
// {
//     // sumbitting agv    
//     if (!shipment_submit_client.exists())
//     {
//         shipment_submit_client.waitForExistence();
//     }

//     ROS_INFO("Requesting the service...");
//     nist_gear::AGVToAssemblyStation srv_msg;
//     srv_msg.request.assembly_station_name = assembly_station;
//     srv_msg.request.shipment_type = shipment_type;

//     ros::Duration(2).sleep();
//     shipment_submit_client.call(srv_msg);

//     if (!srv_msg.response.success)
//     {
//         ROS_ERROR_STREAM("Failed to submit "<< agv_name <<" to assembly station " << srv_msg.request.assembly_station_name);
//     }
//     else
//     {
//         ROS_INFO_STREAM(agv_id << " moving to: " << srv_msg.request.assembly_station_name);
//     }
// }

/////////////////////////////////////// Subscriber Callbacks ///////////////////////////////////////////////////
void AGV::logical_camera_as1_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        as1_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as1_parts.push_back(logicam_msg->models[i]);
            as1_parts[i].pose = transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_as1_frame", "as1");

        }
        as1_parts_count = as1_parts.size();
        ROS_INFO_STREAM_ONCE("Assembly Station 1 part count : " << as1_parts_count);
        // ROS_INFO_STREAM_ONCE("Bins 0 to 3 Parts : " << as1_parts[0]);

    }
}

void AGV::logical_camera_as2_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        as2_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as2_parts.push_back(logicam_msg->models[i]);
            as2_parts[i].pose = transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_as2_frame", "as2");

        }
        as2_parts_count = as1_parts.size();
        ROS_INFO_STREAM_ONCE("Assembly Station 2 part count : " << as2_parts_count);
        ROS_INFO_STREAM_ONCE("Bins 0 to 3 Parts : " << as2_parts[0]);

    }
}

void AGV::logical_camera_as3_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        as3_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as3_parts.push_back(logicam_msg->models[i]);
            as3_parts[i].pose = transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_as3_frame", "as3");

        }
        as3_parts_count = as3_parts.size();
        ROS_INFO_STREAM_ONCE("Assembly Station 3 part count : " << as3_parts_count);
        ROS_INFO_STREAM_ONCE("Bins 0 to 3 Parts : " << as3_parts[0]);

    }
}

void AGV::logical_camera_as4_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        as4_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as4_parts.push_back(logicam_msg->models[i]);
            as4_parts[i].pose = transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_as4_frame", "as4");

        }
        as4_parts_count = as4_parts.size();
        ROS_INFO_STREAM_ONCE("Assembly Station 4 part count : " << as4_parts_count);
        ROS_INFO_STREAM_ONCE("Bins 0 to 3 Parts : " << as4_parts[0]);

    }
}

////////////////////////////////////////////////////
void AGV::logical_camera_as1_agv1_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        as1_agv1_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as1_agv1_parts.push_back(logicam_msg->models[i]);
            as1_agv1_parts[i].pose = transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_as1_agv1_frame", "as1_agv1");

        }
        as1_agv1_parts_count = as1_agv1_parts.size();
        ROS_INFO_STREAM_ONCE("Assembly Station 1, AGV 1 part count : " << as1_agv1_parts_count);
        // ROS_INFO_STREAM_ONCE("Bins 0 to 3 Parts : " << as1_agv1_parts[0]);

    }
}

void AGV::logical_camera_as1_agv2_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        as1_agv2_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as1_agv2_parts.push_back(logicam_msg->models[i]);
            as1_agv2_parts[i].pose = transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_as1_agv2_frame", "as1_agv2");

        }
        as1_agv2_parts_count = as1_agv2_parts.size();
        ROS_INFO_STREAM_ONCE("Assembly Station 1, AGV 2 part count : " << as1_agv2_parts_count);
        ROS_INFO_STREAM_ONCE("Bins 0 to 3 Parts : " << as1_agv2_parts[0]);

    }
}

void AGV::logical_camera_as2_agv1_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        as2_agv1_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as2_agv1_parts.push_back(logicam_msg->models[i]);
            as2_agv1_parts[i].pose = transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_as2_agv1_frame", "as2_agv1");

        }
        as2_agv1_parts_count = as2_agv1_parts.size();
        ROS_INFO_STREAM_ONCE("Assembly Station 3 part count : " << as2_agv1_parts_count);
        ROS_INFO_STREAM_ONCE("Bins 0 to 3 Parts : " << as2_agv1_parts[0]);

    }
}

void AGV::logical_camera_as2_agv2_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        as2_agv2_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as2_agv2_parts.push_back(logicam_msg->models[i]);
            as2_agv2_parts[i].pose = transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_as2_agv2_frame", "as2_agv2");

        }
        as2_agv2_parts_count = as2_agv2_parts.size();
        ROS_INFO_STREAM_ONCE("Assembly Station 4 part count : " << as2_agv2_parts_count);
        ROS_INFO_STREAM_ONCE("Bins 0 to 3 Parts : " << as2_agv2_parts[0]);

    }
}
void AGV::logical_camera_as3_agv3_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        as3_agv3_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as3_agv3_parts.push_back(logicam_msg->models[i]);
            as3_agv3_parts[i].pose = transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_as3_agv3_frame", "as3_agv3");

        }
        as3_agv3_parts_count = as3_agv3_parts.size();
        ROS_INFO_STREAM_ONCE("Assembly Station 1 part count : " << as3_agv3_parts_count);
        // ROS_INFO_STREAM_ONCE("Bins 0 to 3 Parts : " << as3_agv3_parts[0]);

    }
}

void AGV::logical_camera_as3_agv4_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        as3_agv4_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as3_agv4_parts.push_back(logicam_msg->models[i]);
            as3_agv4_parts[i].pose = transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_as3_agv4_frame", "as3_agv4");

        }
        as3_agv4_parts_count = as3_agv4_parts.size();
        ROS_INFO_STREAM_ONCE("Assembly Station 2 part count : " << as3_agv4_parts_count);
        ROS_INFO_STREAM_ONCE("Bins 0 to 3 Parts : " << as3_agv4_parts[0]);

    }
}

void AGV::logical_camera_as4_agv3_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        as4_agv3_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as4_agv3_parts.push_back(logicam_msg->models[i]);
            as4_agv3_parts[i].pose = transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_as4_agv3_frame", "as4_agv3");

        }
        as4_agv3_parts_count = as4_agv3_parts.size();
        ROS_INFO_STREAM_ONCE("Assembly Station 3 part count : " << as4_agv3_parts_count);
        ROS_INFO_STREAM_ONCE("Bins 0 to 3 Parts : " << as4_agv3_parts[0]);

    }
}

void AGV::logical_camera_as4_agv4_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        as4_agv4_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as4_agv4_parts.push_back(logicam_msg->models[i]);
            as4_agv4_parts[i].pose = transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_as4_agv4_frame", "as4_agv4");

        }
        as4_agv4_parts_count = as4_agv4_parts.size();
        ROS_INFO_STREAM_ONCE("Assembly Station 4 part count : " << as4_agv4_parts_count);
        ROS_INFO_STREAM_ONCE("Bins 0 to 3 Parts : " << as4_agv4_parts[0]);

    }
}


geometry_msgs::Pose AGV::transform_to_world_frame(const geometry_msgs::Pose& target_pose, std::string target_frame, std::string child_frame_id) 
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

// bool AGV::agv_details_at_as_service_callback(group2_rwa4::agv_details_at_as::Request &req, group2_rwa4::agv_details_at_as::Response &res)
// {
    
//      if(req.agv_id == "as1_agv1")
//         {
//             for(int i = 0; i < as1_agv1_parts_count; i++)
//             {        
//                 if(as1_agv1_parts[i].type == req.part_type)
//                 {
//                     res.parts_pose_at_agv.push_back(as1_agv1_parts[i].pose);
//                     res.success = true;
//                 }
//             }        
//         }
//         else if(req.agv_id == "as1_agv2")
//         {
//             for(int i = 0; i < as1_agv2_parts_count; i++)
//             {        
//                 if(as1_agv2_parts[i].type == req.part_type)
//                 {
//                     res.parts_pose_at_agv.push_back(as1_agv2_parts[i].pose);
//                     res.success = true;
//                 }
//             }        
//         }
//     else if(req.agv_id =="as2_agv1")
//     {
//         for(int i = 0; i < as2_agv1_parts_count; i++)
//         {        
//             if(as2_agv1_parts[i].type == req.part_type)
//             {
//                 res.parts_pose_at_agv.push_back(as2_agv1_parts[i].pose);
//                 res.success = true;
//             }
//         }        
//     }
//     else if(req.agv_id =="as2_agv2")
//     {
//         for(int i = 0; i < as2_agv2_parts_count; i++)
//         {        
//             if(as2_agv2_parts[i].type == req.part_type)
//             {
//                 res.parts_pose_at_agv.push_back(as2_agv2_parts[i].pose);
//                 res.success = true;
//             }
//         }        
//     }
//     else if(req.agv_id =="as3_agv3")
//     {
//         for(int i = 0; i < as3_agv3_parts_count; i++)
//         {        
//             if(as3_agv3_parts[i].type == req.part_type)
//             {
//                 res.parts_pose_at_agv.push_back(as3_agv3_parts[i].pose);
//                 res.success = true;
//             }
//         }        
//     }
//     else if(req.agv_id =="as3_agv4")
//     {
//         for(int i = 0; i < as3_agv4_parts_count; i++)
//         {        
//             if(as3_agv4_parts[i].type == req.part_type)
//             {
//                 res.parts_pose_at_agv.push_back(as3_agv4_parts[i].pose);
//                 res.success = true;
//             }
//         }        
//     }
//     else if(req.agv_id =="as4_agv3")
//     {
//         for(int i = 0; i < as4_agv3_parts_count; i++)
//         {        
//             if(as4_agv3_parts[i].type == req.part_type)
//             {
//                 res.parts_pose_at_agv.push_back(as4_agv3_parts[i].pose);
//                 res.success = true;
//             }
//         }        
//     }
//     else if(req.agv_id =="as4_agv4")
//     {
//         for(int i = 0; i < as4_agv4_parts_count; i++)
//         {        
//             if(as4_agv4_parts[i].type == req.part_type)
//             {
//                 res.parts_pose_at_agv.push_back(as4_agv4_parts[i].pose);
//                 res.success = true;
//             }
//         }        
//     }
    
//     return true;
// }