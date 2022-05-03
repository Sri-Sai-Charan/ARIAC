#include "sensor_array.h"

//##############################################################
// Conveyor Sensor
//##############################################################

sensor_array::ConveyorSensors::ConveyorSensors(ros::NodeHandle &node_handler)
    {
        nh = node_handler;    
        init();
    }

    
void sensor_array::ConveyorSensors::init()
    {
        logical_camera_0_sub = nh.subscribe("/ariac/logical_camera_0", 10, &sensor_array::ConveyorSensors::logical_camera_0_callback, this);
        // laser_profiler_0_sub = nh.subscribe("/ariac/laser_profiler_0", 10, &sensor_array::ConveyorSensors::laser_profiler_0_callback, this);
        // proximity_sensor_0_sub = nh.subscribe("/ariac/proimity_sensor_0", 10, &sensor_array::ConveyorSensors::proximity_sensor_0_callback, this);
        // breakbeam_0_sub = nh.subscribe("/ariac/breakbeam_0_sub", 10, &sensor_array::ConveyorSensors::breakbeam_0_callback, this);
        get_all_parts_on_conveyor_service = nh.advertiseService("/group2/list_all_parts_conveyor", &sensor_array::ConveyorSensors::get_all_parts_on_conveyor_service_callback, this);

    }

void sensor_array::ConveyorSensors::logical_camera_0_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
    {
        if(logicam_msg->models.size() > 0)
        {
            conveyor_parts.clear();
            for(int i = 0; i < logicam_msg->models.size(); i++)
            {            
                conveyor_parts.push_back(logicam_msg->models[i]);
                conveyor_parts[i].pose = sensor_array::transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_0_frame");

                // ROS_INFO_STREAM_ONCE("Parts on Conveyor: " << conveyor_parts[i]);
            }
            
        }
    }


bool sensor_array::ConveyorSensors::get_all_parts_on_conveyor_service_callback(group2_rwa4::list_all_parts::Request &req, group2_rwa4::list_all_parts::Response &res)
    {
        
        for(int i = 0; i<conveyor_parts.size(); i++){

            res.all_parts.push_back(conveyor_parts[i].pose);
            res.type.push_back(conveyor_parts[i].type);
            ROS_INFO_STREAM("Conveyor service working" << conveyor_parts[i]);
        }
        return true;
    }

// #############################################################
// Bins Sensor
//##############################################################

sensor_array::BinSensors::BinSensors(ros::NodeHandle &node_handler)
    {
        nh = node_handler;    
        init();
    }

void sensor_array::BinSensors::init()
    {
        logicam_bin0_sub = nh.subscribe("/ariac/logical_camera_1", 10, &sensor_array::BinSensors::logicam_bins0_callback, this);
        logicam_bin1_sub = nh.subscribe("/ariac/logical_camera_2", 10, &sensor_array::BinSensors::logicam_bins1_callback, this);
        get_all_parts_on_bins_service = nh.advertiseService("/group2/list_all_parts_bins", &sensor_array::BinSensors::get_all_parts_on_bins_service_callback, this);

    }

void sensor_array::BinSensors::logicam_bins0_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        bins0_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            bins0_parts.push_back(logicam_msg->models[i]);
            bins0_parts[i].pose = sensor_array::transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_1_frame");

        }
        bins0_parts_count = bins0_parts.size();
        ROS_INFO_STREAM_ONCE("Bin 0 part count : " << bins0_parts_count);
        ROS_INFO_STREAM_ONCE("Bins 0 to 3 Parts : " << bins0_parts[0]);

    }
}

void sensor_array::BinSensors::logicam_bins1_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    if(logicam_msg->models.size() > 0)
    {
        bins1_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            bins1_parts.push_back(logicam_msg->models[i]);
            bins1_parts[i].pose = sensor_array::transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_2_frame");

        }
        bins1_parts_count = bins1_parts.size();
        ROS_INFO_STREAM_ONCE("Bin 1 part count : " << bins1_parts_count);
        ROS_INFO_STREAM_ONCE("Bins 3 to 7 Parts : " << bins1_parts[0]);

    }
}

bool sensor_array::BinSensors::get_all_parts_on_bins_service_callback(group2_rwa4::list_all_parts::Request &req, group2_rwa4::list_all_parts::Response &res){
    ROS_INFO_STREAM("Bin service working");
    if (req.request == "1"){
        res.success = true;
        bins_parts = bins0_parts;
        bins_parts.insert(bins_parts.end(), bins1_parts.begin(), bins1_parts.end());
        for(int i = 0; i<bins_parts.size(); i++){
            res.all_parts.push_back(bins_parts[i].pose);
            res.type.push_back(bins_parts[i].type);
            ROS_INFO_STREAM("Bin service working" << bins_parts[i]);
        }
        
        return true;
    }
    return false;
}


// #############################################################
// Agv Sensor
//##############################################################
sensor_array::AgvSensors::AgvSensors(ros::NodeHandle &node_handler)
    {
        nh = node_handler;
        init();
    }

void sensor_array::AgvSensors::init()
    {
        logical_camera_3_sub = nh.subscribe("/ariac/logical_camera_3", 10, &sensor_array::AgvSensors::logical_camera_3_callback, this);
        logical_camera_4_sub = nh.subscribe("/ariac/logical_camera_4", 10, &sensor_array::AgvSensors::logical_camera_4_callback, this);
        logical_camera_5_sub = nh.subscribe("/ariac/logical_camera_4", 10, &sensor_array::AgvSensors::logical_camera_5_callback, this);
        logical_camera_6_sub = nh.subscribe("/ariac/logical_camera_4", 10, &sensor_array::AgvSensors::logical_camera_6_callback, this);
        quality_control_sensor_1_sub = nh.subscribe("/ariac/quality_control_sensor_1", 10, &sensor_array::AgvSensors::quality_control_sensor_1_callback, this);
        quality_control_sensor_2_sub = nh.subscribe("/ariac/quality_control_sensor_2", 10, &sensor_array::AgvSensors::quality_control_sensor_2_callback, this);
        quality_control_sensor_3_sub = nh.subscribe("/ariac/quality_control_sensor_3", 10, &sensor_array::AgvSensors::quality_control_sensor_3_callback, this);
        quality_control_sensor_4_sub = nh.subscribe("/ariac/quality_control_sensor_4", 10, &sensor_array::AgvSensors::quality_control_sensor_4_callback, this);
        get_all_parts_on_agv_logical_cam_service = nh.advertiseService("/group2/list_all_parts_agv_lcam", &sensor_array::AgvSensors::get_all_parts_on_agv_logical_cam_service_callback, this);
        get_all_parts_on_agv_qc_sensor_service = nh.advertiseService("/group2/list_all_parts_agv_qc", &sensor_array::AgvSensors::get_all_parts_on_agv_qc_sensor_service_callback, this);

    }


// // ###################### QC SENSOR AGV #######################################

void sensor_array::AgvSensors::quality_control_sensor_1_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg)
{

    if(msg->models.size() > 0)
    {
        ROS_INFO_STREAM("Faulty part detected on AGV1:" << msg->models.size());
        faulty_agv1_parts.clear();
        for(int i = 0; i < msg->models.size(); i++)
        {            
            faulty_agv1_parts.push_back(msg->models[i]);
            faulty_agv1_parts[i].pose = sensor_array::transform_to_world_frame(msg->models[i].pose,"quality_control_sensor_1_frame");

        }
        
    }      

};

void sensor_array::AgvSensors::quality_control_sensor_2_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg)
{

    if(msg->models.size() > 0)
    {
        
        faulty_agv2_parts.clear();
        for(int i = 0; i < msg->models.size(); i++)
        {            
            faulty_agv2_parts.push_back(msg->models[i]);
            faulty_agv2_parts[i].pose = sensor_array::transform_to_world_frame(msg->models[i].pose,"quality_control_sensor_2_frame");

        }
    }      

};


void sensor_array::AgvSensors::quality_control_sensor_3_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg)
{

    if(msg->models.size() > 0)
    {
        
        faulty_agv3_parts.clear();
        for(int i = 0; i < msg->models.size(); i++)
        {            
            faulty_agv3_parts.push_back(msg->models[i]);
            faulty_agv3_parts[i].pose = sensor_array::transform_to_world_frame(msg->models[i].pose,"quality_control_sensor_3_frame");

        }
    }
    
};

void sensor_array::AgvSensors::quality_control_sensor_4_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg)
{

    if(msg->models.size() > 0)
    {
        
        faulty_agv4_parts.clear();
        for(int i = 0; i < msg->models.size(); i++)
        {            
            faulty_agv4_parts.push_back(msg->models[i]);
            faulty_agv4_parts[i].pose = sensor_array::transform_to_world_frame(msg->models[i].pose,"quality_control_sensor_4_frame");

        }
    }      

};

bool sensor_array::AgvSensors::get_all_parts_on_agv_qc_sensor_service_callback(group2_rwa4::list_all_parts::Request &req, group2_rwa4::list_all_parts::Response &res)
{
   
    if (req.request == "1"){
        res.success = true;
        for(int i = 0; i<faulty_agv1_parts.size(); i++){

            res.all_parts.push_back(faulty_agv1_parts[i].pose);
            res.type.push_back(faulty_agv1_parts[i].type);
            
        }
        
        return true;
    }
    
    if (req.request == "2"){
        res.success = true;
        for(int i = 0; i<faulty_agv2_parts.size(); i++){
            
            res.all_parts.push_back(faulty_agv2_parts[i].pose);
            res.type.push_back(faulty_agv2_parts[i].type);
            
        }
        
        return true;
    }

    if (req.request == "3"){
        res.success = true;
        for(int i = 0; i<faulty_agv3_parts.size(); i++){
            
            res.all_parts.push_back(faulty_agv3_parts[i].pose);
            res.type.push_back(faulty_agv3_parts[i].type);
            
        }
        
        return true;
    }

    if (req.request == "4"){
        res.success = true;
        for(int i = 0; i<agv4_parts.size(); i++){

            res.all_parts.push_back(agv4_parts[i].pose);
            res.type.push_back(agv4_parts[i].type);
            
        }
        
        return true;
    }
    return false;
}





// // ###################### Logical Camera on  AGV #######################################


void sensor_array::AgvSensors::logical_camera_3_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    
    if(logicam_msg->models.size() > 0)
    {
        agv1_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            agv1_parts.push_back(logicam_msg->models[i]);            
            agv1_parts[i].pose = sensor_array::transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_3_frame");
        }
        agv1_parts_count = agv1_parts.size();
    }
}

void sensor_array::AgvSensors::logical_camera_4_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    // ROS_INFO_STREAM_ONCE(*logicam_msg);
    if(logicam_msg->models.size() > 0)
    {
        agv2_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            agv2_parts.push_back(logicam_msg->models[i]);
            agv2_parts[i].pose = sensor_array::transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_4_frame");

        }
        agv2_parts_count = agv2_parts.size();
    }
}


void sensor_array::AgvSensors::logical_camera_5_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    // ROS_INFO_STREAM_ONCE(*logicam_msg);
    if(logicam_msg->models.size() > 0)
    {
        agv3_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            agv3_parts.push_back(logicam_msg->models[i]);
            agv3_parts[i].pose = sensor_array::transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_5_frame");

        }
        agv3_parts_count = agv3_parts.size();
    }
}

void sensor_array::AgvSensors::logical_camera_6_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    // ROS_INFO_STREAM_ONCE(*logicam_msg);
    if(logicam_msg->models.size() > 0)
    {
        agv4_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            agv4_parts.push_back(logicam_msg->models[i]);
            agv4_parts[i].pose = sensor_array::transform_to_world_frame(logicam_msg->models[i].pose,"logical_camera_6_frame");

        }
        agv4_parts_count = agv4_parts.size();
    }
}



bool sensor_array::AgvSensors::get_all_parts_on_agv_logical_cam_service_callback(group2_rwa4::list_all_parts::Request &req, group2_rwa4::list_all_parts::Response &res)
{
    
    if (req.request == "1"){
        res.success = true;
        for(int i = 0; i<agv1_parts.size(); i++){

            res.all_parts.push_back(agv1_parts[i].pose);
            res.type.push_back(agv1_parts[i].type);
            
        }
        
        return true;
    }
    
    if (req.request == "2"){
        res.success = true;
        for(int i = 0; i<agv2_parts.size(); i++){
            
            res.all_parts.push_back(agv2_parts[i].pose);
            res.type.push_back(agv2_parts[i].type);
            
        }
        
        return true;
    }

    if (req.request == "3"){
        res.success = true;
        for(int i = 0; i<agv3_parts.size(); i++){
            
            res.all_parts.push_back(agv3_parts[i].pose);
            res.type.push_back(agv3_parts[i].type);
            
        }
        
        return true;
    }

    if (req.request == "4"){
        res.success = true;
        for(int i = 0; i<agv4_parts.size(); i++){

            res.all_parts.push_back(agv4_parts[i].pose);
            res.type.push_back(agv4_parts[i].type);
        }
        
        return true;
    }
    return false;

}


// ############################################## Transform ########################################


geometry_msgs::Pose sensor_array::transform_to_world_frame(const geometry_msgs::Pose& part_pose, std::string logicam_frame) 
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




// #############################################################
// Assembly Sensor
//##############################################################