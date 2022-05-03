#include "assembly_handler.h"


AssemblyHandler::AssemblyHandler(ros::NodeHandle &node_handler)
{
    nh = node_handler;
    init();
}

void AssemblyHandler::init()
{
    ROS_INFO("AssemblyHandler initialized :)");

    order_assembly_shipment_client = nh.serviceClient<group2_rwa4::order_assembly_shipment_details>("/group2/get_order_assembly_shipment_details");
    order_completion_status_client = nh.serviceClient<group2_rwa4::order_completion_status>("/group2/order_completion_status");   
    gantry_robot_pick_place_client = nh.serviceClient<group2_rwa4::assembly_part_details>("/group2/gantry_robot_pick_and_place");

    assembly_task_client = nh.serviceClient<group2_rwa4::assembly_task>("/group2/assembly_task");

    assembly_cam_subs.resize(8);
    for (int i = 0; i < 8; i++)
    {
        assembly_cam_subs.at(i) = nh.subscribe<nist_gear::LogicalCameraImage>("/ariac/logical_camera_1"+std::to_string(i+1),1,boost::bind(&AssemblyHandler::logical_camera_sub_callback,this,_1,i));
    }
    perform_part_assembly();
}


void AssemblyHandler::logical_camera_sub_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg,int cam_id)
{
    if (logicam_msg->models.size()>0)
    {
        assembly_logical_cams_data[cam_id].clear();
        for (int i = 0; i < logicam_msg->models.size(); i++)
        {
            assembly_logical_cams_data[cam_id].push_back(logicam_msg->models[i]);
        }        
    }
}


int AssemblyHandler::find_part(assembly_part_location *current_part,std::string required_part,std::string final_preset_location){
    int start;
    int end;
    if (final_preset_location == "as1")
    {
        start = 0;
        end = 2;
    }else if (final_preset_location == "as2")
    {
        start =4;
        end =6;        
    }else if (final_preset_location == "as3")
    {
        start = 2;
        end = 4;
    }else if(final_preset_location == "as4")
    {
        start =6;
        end =8;
    }
    
    
    for (int i = start; i < end; i++)
    {
        for (int j = 0; j < assembly_logical_cams_data[i].size(); j++)
        {
           if (required_part == assembly_logical_cams_data[i].at(j).type)
           {
               current_part->part_pose = assembly_logical_cams_data[i].at(j).pose;
               switch (i)
               {
                case 0:
                   current_part->initial_preset_location = "as1_agv1";
                   return 0;
                case 1:
                    current_part->initial_preset_location = "as1_agv2";
                    return 0;
                case 2:
                    current_part->initial_preset_location = "as3_agv3";
                    return 0;
                case 3:
                    current_part->initial_preset_location = "as3_agv4";
                    return 0;
                case 4:
                    current_part->initial_preset_location = "as2_agv1";
                    return 0;
                case 5:
                    current_part->initial_preset_location = "as2_agv2";
                    return 0;
                case 6:
                    current_part->initial_preset_location = "as4_agv3";
                    return 0;
                case 7:
                    current_part->initial_preset_location = "as4_agv4";
                    return 0;
               }
               
           }
           
        }
        
    }
    ROS_INFO("parts not found");
    return 0;
    
}



void AssemblyHandler::perform_part_assembly()
{
    if(!gantry_robot_pick_place_client.waitForExistence())
    {
        gantry_robot_pick_place_client.exists();
    }
    if(!assembly_task_client.waitForExistence())
    {
        assembly_task_client.exists();
    }
    group2_rwa4::assembly_task as_task_msg;
    group2_rwa4::kitting_part_details gantry_msg;
    while (true)
    {
        as_task_msg.request.request = task_current;
        assembly_task_client.call(as_task_msg);

        if (as_task_msg.response.sucess)
        {
            gantry_msg.request.final_preset_location = as_task_msg.response.final_preset_location;
            gantry_msg.request.part_offset = as_task_msg.response.part_offset;
            gantry_msg.request.part_target_pose = as_task_msg.response.part_target_pose;
            int temp =find_part(&location_,as_task_msg.response.part_type,as_task_msg.response.final_preset_location);
            ROS_INFO_STREAM("Inital Preset :" << location_.initial_preset_location);
            // gantry_msg.request.initial_preset_location = 
            ros::Duration(3).sleep();
            task_current+=1;
            ROS_INFO("Next Task requested");

        }
        if (as_task_msg.response.part_type=="Task done")
        {
            ROS_INFO("Task Done");
            break;
        }
        
    }
    
    
    

    // if(assembly_task.task_type == "assembly")
    // {
    //     group2_rwa4::assembly_part_details assembly_part_details_srv_msg;
        
    //     assembly_part_details_srv_msg.request.initial_preset_location = assembly_task.initial_preset_location;
    //     assembly_part_details_srv_msg.request.final_preset_location = assembly_task.final_preset_location;
    //     assembly_part_details_srv_msg.request.part_pose = assembly_task.part_pose;
    //     assembly_part_details_srv_msg.request.part_target_pose = assembly_task.part_target_pose;

    //     std::string part_type = assembly_task.part_type;
    //     std::string delimiter = "_";
    //     std::vector<std::string> split_words{};

    //     std::size_t pos;
    //     while ((pos = part_type.find(delimiter)) != std::string::npos) {
    //         split_words.push_back(part_type.substr(0, pos));
    //         part_type.erase(0, pos + delimiter.length());
    //     }

    //     std::string part_name = split_words[1];

    //     // setting part offset pose value
    //     if(part_name == "battery")
    //     {
    //         assembly_part_details_srv_msg.request.part_offset = 0.045;
    //     } 
    //     else if(part_name == "sensor")
    //     {
    //         assembly_part_details_srv_msg.request.part_offset = 0.053;
    //     }
    //     else if(part_name == "regulator")
    //     {
    //         assembly_part_details_srv_msg.request.part_offset = 0.06;
    //     }
    //     else
    //     {
    //         assembly_part_details_srv_msg.request.part_offset = 0.08;
    //     } 


    //     if(!gantry_robot_pick_place_client.call(assembly_part_details_srv_msg))
    //     {
    //         ROS_INFO("Retrying part pick and place");
    //         gantry_robot_pick_place_client.call(assembly_part_details_srv_msg);
    //     }

    //     ROS_INFO("Assembly done!!"); 

    }

