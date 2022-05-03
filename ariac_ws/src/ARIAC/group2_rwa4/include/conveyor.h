#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <nist_gear/AGVToAssemblyStation.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Model.h>
#include <group2_rwa4/check_exists.h>
#include <group2_rwa4/check_part_pose.h>

#include <nist_gear/Proximity.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <std_srvs/Trigger.h>


class Conveyor{

    private:
    ros::NodeHandle nh;
    ros::Subscriber break_beam_sub;
    ros::Subscriber logical_camera_0_sub;
    geometry_msgs::Pose part_pose;
    ros::Publisher conveyor_part_pose_pub;


    public:    

    Conveyor(ros::NodeHandle &node_handler);

    void init();

    void break_beam_sub_callback(const nist_gear::Proximity::ConstPtr &msg);

    void logical_camera_0_sub_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);

    geometry_msgs::Pose transform_to_world_frame(const geometry_msgs::Pose& part_pose, std::string logicam_frame);
};

