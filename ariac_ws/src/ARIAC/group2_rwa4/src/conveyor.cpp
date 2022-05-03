#include "conveyor.h"

Conveyor::Conveyor(ros::NodeHandle &node_handler)
{
    nh = node_handler;    
    init();
}

void Conveyor::init()
{   
    break_beam_sub = nh.subscribe("/ariac/breakbeam_0", 10, &Conveyor::break_beam_sub_callback, this);
    logical_camera_0_sub = nh.subscribe("/ariac/logical_camera_0", 10, &Conveyor::logical_camera_0_sub_callback, this);
    conveyor_part_pose_pub = nh.advertise<geometry_msgs::Pose>("/group2/conveyor_part_pose", 1000, true);

    // publishing empty pose while initialized
    geometry_msgs::Pose empty_pose;
    conveyor_part_pose_pub.publish(empty_pose);
}


void Conveyor::break_beam_sub_callback(const nist_gear::Proximity::ConstPtr &msg)
{
    //ROS_INFO_STREAM_ONCE("break_beam_sub working");
    if(msg-> object_detected)
    {
        //ROS_INFO("Object detected on conveyor");
        ros::ServiceClient create_conveyor_task_srv = nh.serviceClient<std_srvs::Trigger>("/group2/create_conveyor_task");

        std_srvs::Trigger create_conveyor_task_srv_msg;
        create_conveyor_task_srv.call(create_conveyor_task_srv_msg);

        //ROS_INFO_STREAM(create_conveyor_task_srv_msg.response);
    }
}

void Conveyor::logical_camera_0_sub_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    //ROS_INFO_STREAM_ONCE("Logicam conveyor working");
    if(logicam_msg->models.size() > 0)
    {
        // while(logicam_msg->models)
        geometry_msgs::Pose part_pose = logicam_msg->models[0].pose;
        geometry_msgs::Pose world_pose = transform_to_world_frame(part_pose, "logical_camera_0_frame");
        conveyor_part_pose_pub.publish(world_pose);

    }
    else
    {
        geometry_msgs::Pose empty_pose;
        conveyor_part_pose_pub.publish(empty_pose);
    }
}


geometry_msgs::Pose Conveyor::transform_to_world_frame(const geometry_msgs::Pose& part_pose, std::string logicam_frame) 
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

// void OrderHandler::publish_world_pose_on_conveyoy()
// {
//     ros::Rate loop_rate(1);
//     while (ros::ok())
//     {        
//         if(kitting_task_queue.size() > 0)
//         {
//             //ROS_INFO("Publishing task");
//             task_pub.publish(kitting_task_queue[0]);
//         }        

//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }
