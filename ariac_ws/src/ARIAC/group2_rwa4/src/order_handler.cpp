#include "order_handler.h"


OrderHandler::OrderHandler(ros::NodeHandle &nodehandler)
{
    nh = nodehandler;
    init();
}

void OrderHandler::init()
{
    // wait till competion has started
    bool is_competition_started = false;
    nh.getParam("/is_competition_started", is_competition_started);
    while (!is_competition_started)
    {
        nh.getParam("/is_competition_started", is_competition_started);
    }

    orders_sub = nh.subscribe("/ariac/orders", 10, &OrderHandler::process_orders, this);
    task_pub = nh.advertise<group2_rwa4::Task>("/group2/kitting_task", 1, true);
    order_details_service = nh.advertiseService("/group2/get_order_details", &OrderHandler::order_details_service_callback, this);
    order_kitting_shipment_details_service = nh.advertiseService("/group2/get_order_kitting_shipment_details", &OrderHandler::order_kitting_shipment_details_service_callback, this);

    order_completion_status_service = nh.advertiseService("/group2/order_completion_status", &OrderHandler::order_completion_status_service_callback, this);

    logicam_bins0_client = nh.serviceClient<group2_rwa4::check_exists>("/group2/check_part_bins0");
    logicam_bins1_client = nh.serviceClient<group2_rwa4::check_exists>("/group2/check_part_bins1");


    task_completed_srv = nh.advertiseService("/group2/task_completed", &OrderHandler::task_completed_srv_callback, this);
    dispose_faulty_srv = nh.advertiseService("/group2/dispose_faulty", &OrderHandler::dispose_faulty_srv_callback, this);
    create_conveyor_task_srv = nh.advertiseService("/group2/create_conveyor_task", &OrderHandler::create_conveyor_task_srv_callback, this);


    // #################################################Assembly####################################################
    assembly_task_srv = nh.advertiseService("/group2/assembly_task", &OrderHandler::get_next_task_srv_callback, this);

}

void OrderHandler::conveyor_part_callback(const geometry_msgs::Pose &part_pose)
{
    ROS_INFO_STREAM_ONCE("Logicam bin1 working");
    
    if(part_pose.position.y!=0)
    {
        conveyor_pose = part_pose;
    }

}

void OrderHandler::process_orders(const nist_gear::Order::ConstPtr &order)
{   
    
    incomplete_orders.push_back(*order);
    update_kitting_task_queue();
    
    ROS_INFO("New order recieved");   
    // ROS_INFO_STREAM("Order_ID : "<< incomplete_orders[0].order_id);
    nh.setParam("/is_order_initialized", true); 
    
}

int OrderHandler::get_completed_order_count()
{
    return completed_orders.size();
}

int OrderHandler::get_incomplete_order_count()
{
    return incomplete_orders.size();
}

bool OrderHandler::order_details_service_callback(group2_rwa4::list_order_details::Request &req, group2_rwa4::list_order_details::Response &res)
{   
    return true;
}

bool OrderHandler::order_kitting_shipment_details_service_callback(group2_rwa4::order_kitting_shipment_details::Request &req, group2_rwa4::order_kitting_shipment_details::Response &res)
{
    group2_rwa4::KittingShipment kitting_shipment;
    
    if(incomplete_orders.size() > 0)
    {    
        for(int i=0; i< incomplete_orders[0].kitting_shipments.size(); i++)
        {   
            kitting_shipment.shipment_type = incomplete_orders[0].kitting_shipments[i].shipment_type;
            kitting_shipment.agv_id = incomplete_orders[0].kitting_shipments[i].agv_id;
            kitting_shipment.station_id = incomplete_orders[0].kitting_shipments[i].station_id;

            group2_rwa4::Product product;
            for(int j=0; j< incomplete_orders[0].kitting_shipments[i].products.size(); j++)
            {   
                product.type = incomplete_orders[0].kitting_shipments[i].products[j].type;
                product.pose = incomplete_orders[0].kitting_shipments[i].products[j].pose;
                product.part_kitting_success = false;
                kitting_shipment.products.push_back(product);
            }
            res.kitting_shipments.push_back(kitting_shipment);
            
        }
        res.order_id = incomplete_orders[0].order_id;
        res.kitting_shipment_present = true;
    }
    else
    {
        res.kitting_shipment_present = false;
    }

    return true;
}


bool OrderHandler::order_completion_status_service_callback(group2_rwa4::order_completion_status::Request &req, group2_rwa4::order_completion_status::Response &res)
{
    //ROS_INFO_STREAM(req);
    if(req.completion_status)
    {
        for(int i=0; i< incomplete_orders.size(); i++)
        {
            if(req.order_id == incomplete_orders[i].order_id)
            {
                incomplete_orders.erase(incomplete_orders.begin()+i);
                res.success = true;

                // ROS_INFO_STREAM(req.order_id << " successful completed :)");
            }
            
        }
    }

    res.success = false;

    return true;
}

void OrderHandler::update_kitting_task_queue()
{
    if(incomplete_orders.size() > 0)
    {    
        group2_rwa4::Task task;

        // looping thru kitting shipment products and creating kitting tasks
        for(int i=0; i< incomplete_orders[0].kitting_shipments.size(); i++)
        {   
            for(int j=0; j< incomplete_orders[0].kitting_shipments[i].products.size(); j++)
            { 
                // ROS_INFO("Adding task");
                task.task_id = kitting_task_id;
                task.task_priority = 1;
                task.task_type = "kitting";
                task.part_type = incomplete_orders[0].kitting_shipments[i].products[j].type;

                // checking bins for parts and getting the pose
                group2_rwa4::check_exists bins0_srv_msg;
                group2_rwa4::check_exists bins1_srv_msg;

                bins0_srv_msg.request.part_type = task.part_type;
                logicam_bins0_client.call(bins0_srv_msg);

                bins1_srv_msg.request.part_type = task.part_type;
                logicam_bins1_client.call(bins1_srv_msg);

                if (bins0_srv_msg.response.success)
                {
                    task.initial_preset_location = "bins0";
                    if(kitting_task_queue.size() > 0)
                    {
                        for(int k=0; k< kitting_task_queue.size(); k++)
                        {                              
                            for(int m=0; m< bins0_srv_msg.response.part_poses.size(); m++)
                            {  
                                if(kitting_task_queue[k].part_pose != bins0_srv_msg.response.part_poses[m])
                                {
                                    ROS_INFO("Part pose added");
                                    task.part_pose = bins0_srv_msg.response.part_poses[m];
                                    break;
                                }
                            }  
                        }   
                    }
                    else
                    {
                        task.part_pose = bins0_srv_msg.response.part_poses[0];
                    }                       
                }
                else if (bins1_srv_msg.response.success)
                {
                    task.initial_preset_location = "bins1";
                    if(kitting_task_queue.size() > 0)
                    {
                        for(int k=0; k< kitting_task_queue.size(); k++)
                        {                              
                            for(int m=0; m< bins1_srv_msg.response.part_poses.size(); m++)
                            {  
                                if(kitting_task_queue[k].part_pose != bins1_srv_msg.response.part_poses[m])
                                {
                                    ROS_INFO("Part pose added");
                                    task.part_pose = bins1_srv_msg.response.part_poses[m];
                                    break;
                                }
                            }  
                        }   
                    }
                    else
                    {
                        task.part_pose = bins1_srv_msg.response.part_poses[0];
                    } 

                }

                task.final_preset_location = incomplete_orders[0].kitting_shipments[i].agv_id;
                geometry_msgs::Pose desired_part_pose_in_agv = incomplete_orders[0].kitting_shipments[i].products[j].pose;

                int kit_tray_number = 1;
                if(task.final_preset_location == "agv2")
                {
                    kit_tray_number = 2;
                }
                else if(task.final_preset_location == "agv3")
                {
                    kit_tray_number = 3;
                }
                else if(task.final_preset_location == "agv4")
                {
                    kit_tray_number = 4;
                }
                else{} 
                std::string target_frame = "kit_tray_"+std::to_string(kit_tray_number); 
                task.part_target_pose = get_part_target_pose(desired_part_pose_in_agv, target_frame, "part_frame"+i);

                // pushing the task to task queue
                kitting_task_queue.push_back(task); 
                kitting_task_id += 1;          
            }
        }        
        
        ROS_INFO("Publishing first task");
        task_pub.publish(kitting_task_queue[0]);
    }
}


bool OrderHandler::get_next_task_srv_callback(group2_rwa4::assembly_task::Request &req, group2_rwa4::assembly_task::Response &res)
{
    if(incomplete_orders.size() > 0)
    {    
        group2_rwa4::Task task;
        assembly_task_queue.clear();
        for (int idx = 0; idx < incomplete_orders.size(); idx++)
        {
            for(int i=0; i< incomplete_orders[idx].assembly_shipments.size(); i++)
            {   
                for(int j=0; j< incomplete_orders[idx].assembly_shipments[i].products.size(); j++)
                { 
                    task.task_id = assembly_task_id;
                    task.task_priority = 1;
                    task.task_type = "assembly";
                    task.part_type = incomplete_orders[idx].assembly_shipments[i].products[j].type;               
                    task.part_target_pose = incomplete_orders[idx].assembly_shipments[i].products[j].pose;
                    task.final_preset_location = incomplete_orders[idx].assembly_shipments[i].station_id;
                    assembly_task_queue.push_back(task); 
                    assembly_task_id += 1;          
                }
            }                    
        }

        if (req.request >= assembly_task_queue.size())
            {
                res.sucess = false;
                res.part_type = "Task_init";
                if (assembly_task_queue.size()>0)
                {
                    res.part_type = "Task done";
                }
                
                // return false;
            }else
            {
                res.final_preset_location = assembly_task_queue.at(req.request).final_preset_location;
                // res.part_offset = assembly_task_queue.at(req.request).part_offest;
                res.part_type = assembly_task_queue.at(req.request).part_type;
                res.part_target_pose = assembly_task_queue.at(req.request).part_target_pose;
                res.sucess = true;
                // return true;
            }
    }
    return true;

}



bool OrderHandler::task_completed_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{   

    if(kitting_task_queue.size() > 0)
    {
        group2_rwa4::Task completed_task = kitting_task_queue[0];

        kitting_task_queue.erase(kitting_task_queue.begin());     
        
        if(kitting_task_queue.size() > 0)
        {
            res.success = true;
            res.message = "task changed";
            ROS_INFO("Publishing next task");
            task_pub.publish(kitting_task_queue[0]);               
        }    
        else
        {
            res.success = false;
            res.message = "task not found";
        }
    }
    else 
    {   
        res.success = false;
        res.message = "task not found";
    }
    
    // check if agv is ready to submit
    check_agv_ready();
    return true;
}


bool OrderHandler::dispose_faulty_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{   

    if(kitting_task_queue.size() > 0)
    {
        group2_rwa4::Task faulty_part_task = kitting_task_queue[0];
        faulty_part_task.task_id = kitting_task_id;
        kitting_task_id += 1;
        faulty_part_task.task_type = "dispose_faulty";

        // fetching the world pose of faulty part to dispose
        std::string agv_id =faulty_part_task.final_preset_location;
        ros::ServiceClient agv_qc_client = nh.serviceClient<group2_rwa4::check_agv_faulty_parts>("/group2/check_"+agv_id+"_faulty_parts");
        group2_rwa4::check_agv_faulty_parts agv_faulty_part_srv_msg;
        agv_faulty_part_srv_msg.request.agv_id = agv_id;
        agv_qc_client.call(agv_faulty_part_srv_msg);

        int sensor_id = 4;
        if(faulty_part_task.initial_preset_location == "agv1")
        {
            sensor_id = 1;

        } else if(faulty_part_task.initial_preset_location == "agv2")
        {
            sensor_id = 2;

        } else if(faulty_part_task.initial_preset_location == "agv3")
        {
            sensor_id = 3;

        } else {}


        std::string agv_qc_frame = "quality_control_sensor_"+std::to_string(sensor_id)+"_frame";
        geometry_msgs::Pose qc_faulty_part_pose = agv_faulty_part_srv_msg.response.faulty_part_poses[0];
        geometry_msgs::Pose faulty_part_world_pose = get_part_target_pose(qc_faulty_part_pose, agv_qc_frame, "faulty_frame"); 

        faulty_part_task.initial_preset_location = faulty_part_task.final_preset_location;
        faulty_part_task.final_preset_location = "trash_bin";
        faulty_part_task.part_pose = faulty_part_world_pose;

        // checking bins for parts and getting the pose of replacement part
        group2_rwa4::Task replacement_task = kitting_task_queue[0];
        group2_rwa4::check_exists bins0_srv_msg;
        group2_rwa4::check_exists bins1_srv_msg;

        bins0_srv_msg.request.part_type = kitting_task_queue[0].part_type;
        logicam_bins0_client.call(bins0_srv_msg);

        bins1_srv_msg.request.part_type = kitting_task_queue[0].part_type;
        logicam_bins1_client.call(bins1_srv_msg);

        if (bins0_srv_msg.response.success)
        {
            replacement_task.initial_preset_location = "bins0";
            for(int k=0; k< kitting_task_queue.size(); k++)
            {                  
                for(int m=0; m< bins0_srv_msg.response.part_poses.size(); m++)
                {  
                    if(kitting_task_queue[k].part_pose != bins0_srv_msg.response.part_poses[m])
                    {
                        replacement_task.part_pose = bins0_srv_msg.response.part_poses[m];
                    }
                }  
            }  
        }
        else if (bins1_srv_msg.response.success)
        {
            replacement_task.initial_preset_location = "bins1";
            for(int k=0; k< kitting_task_queue.size(); k++)
            {                  
                for(int m=0; m< bins1_srv_msg.response.part_poses.size(); m++)
                {  
                    if(kitting_task_queue[k].part_pose != bins1_srv_msg.response.part_poses[m])
                    {
                        replacement_task.part_pose = bins1_srv_msg.response.part_poses[m];
                    }
                }  
            }  
        }        

        // removing old task and adding dispose task + part replacement task
        kitting_task_queue.erase(kitting_task_queue.begin()); 
        kitting_task_queue.insert(kitting_task_queue.begin(), faulty_part_task); 
        kitting_task_queue.insert(kitting_task_queue.begin()+1, replacement_task);
        
        if(kitting_task_queue.size() > 0)
        {
            res.success = true;
            res.message = "task changed";
            ROS_INFO("Publishing next task");
            task_pub.publish(kitting_task_queue[0]);   
        }    
        else
        {
            res.success = false;
            res.message = "task not found";
        }
    }
    else 
    {
        res.success = false;
        res.message = "task not found";
    }
    return true;
}

void OrderHandler::check_agv_ready()
{
    ROS_INFO("Checking if agv is ready");
    for(int i=0; i< incomplete_orders[0].kitting_shipments.size(); i++)
    { 
        int task_present_count = 0;
        for(int j=0; j< kitting_task_queue.size(); j++)
        {  
            if(kitting_task_queue[j].task_type == "kitting" && kitting_task_queue[j].final_preset_location == incomplete_orders[0].kitting_shipments[i].agv_id)
            {
                task_present_count += 1;
            }                
        } 

        if(task_present_count == 0)
        {
            ROS_INFO_STREAM(incomplete_orders[0].kitting_shipments[i].agv_id << " is ready to ship");
            submit_agv(incomplete_orders[0].kitting_shipments[i].agv_id, 
                    incomplete_orders[0].kitting_shipments[i].station_id, 
                    incomplete_orders[0].kitting_shipments[i].shipment_type);        
            
        }
    }    
    
}

void OrderHandler::submit_agv(std::string agv_id, std::string assembly_station, std::string shipment_type)
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

bool OrderHandler::create_conveyor_task_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{   

    std::string insufficient_part;
    std::string conveyor_part_type;
    nh.getParam("/insufficient_part", insufficient_part);
    nh.getParam("/conveyor_part_type", conveyor_part_type);

    //ROS_INFO("checking Conveyor part pickup needed");

    if(conveyor_part_type == insufficient_part)
    {   
        bool conveyor_pickup_already_present = false;
        for(int i=0; i<kitting_task_queue.size(); i++)
        {
            if(kitting_task_queue[i].task_type == "conveyor_task")
            {
                conveyor_pickup_already_present = true;
            }
        }

        if(!conveyor_pickup_already_present)
        {
            //ROS_INFO("Conveyor part pickup task created");
            group2_rwa4::Task conveyor_task;
            conveyor_task.task_id = kitting_task_id;
            kitting_task_id += 1;
            conveyor_task.task_type = "conveyor_task";
            conveyor_task.initial_preset_location = "conveyor";
            conveyor_task.final_preset_location = "bins0";

            nh.getParam("/conveyor_part_type", conveyor_task.part_type);

            bool is_kitting_robot_busy;
            nh.getParam("/is_kitting_robot_busy", is_kitting_robot_busy);

            if(kitting_task_queue.size() > 0)
            {
                
                kitting_task_queue.insert(kitting_task_queue.begin()+1, conveyor_task);        
                
            }
            else
            {
                kitting_task_queue.insert(kitting_task_queue.begin(), conveyor_task);
                //ROS_INFO("Publishing next task");
                task_pub.publish(kitting_task_queue[0]);  
            }
            

            res.success = true;
            res.message = "conveyor task added";  
        }
    }
    else
    {
        res.success = false;
        res.message = "part not need";  
    }

     

    return true;

}

geometry_msgs::Pose OrderHandler::get_part_target_pose(const geometry_msgs::Pose& target_pose, std::string target_frame, std::string child_frame_id) 
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
