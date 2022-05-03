
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <array>
#include "utils.h"

// // keep the angle range between + -3.14
// double motioncontrol::adjustYaw(double yaw) {
//     auto result = yaw;
//     if (result > 3.14 || result < -3.14) {
//         if (result > 3.14)
//             result -= 6.28;
//         else
//             result += 6.28;
//     }

//     return result;

// }

namespace motioncontrol {
    
    void print(const tf2::Quaternion& quat) {
        ROS_INFO("[x: %f, y: %f, z: %f, w: %f]",
            quat.getX(), quat.getY(), quat.getZ(), quat.getW());
    }

    void print(const geometry_msgs::Pose& pose) {
        auto rpy = eulerFromQuaternion(pose);
        
        ROS_INFO("position: [x: %f, y: %f, z: %f]\norientation(quat): [x: %f, y: %f, z: %f, w: %f]\norientation(rpy): [roll: %f, pitch: %f, yaw: %f]",
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
            rpy[0], rpy[1], rpy[2]);
    }

    tf2::Quaternion quaternionFromEuler(double r, double p, double y) {
        tf2::Quaternion q;
        q.setRPY(r, p, y);
        ROS_INFO("quat: [%f, %f, %f, %f]",
            q.getX(),
            q.getY(),
            q.getZ(),
            q.getW());

        return q;
    }

    std::array<double, 3> eulerFromQuaternion(const tf2::Quaternion& quat) {

        tf2::Quaternion q(
            quat.getX(),
            quat.getY(),
            quat.getZ(),
            quat.getW());
        tf2::Matrix3x3 m(q);


        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        ROS_INFO("[%f, %f, %f]", roll, pitch, yaw);

        std::array<double, 3> rpy_array{ roll, pitch, yaw };
        return rpy_array;
    }

    std::array<double, 3> eulerFromQuaternion(
        const geometry_msgs::Pose& pose) {
        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        tf2::Matrix3x3 m(q);


        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ROS_INFO("[%f, %f, %f]", roll, pitch, yaw);

        std::array<double, 3> rpy_array{ roll, pitch, yaw };
        return rpy_array;
    }

    std::array<double, 3> eulerFromQuaternion(
        double x, double y, double z, double w) {
        tf2::Quaternion q(x, y, z, w);
        tf2::Matrix3x3 m(q);


        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        ROS_INFO("%f, %f, %f", roll, pitch, yaw);

        std::array<double, 3> rpy_array{ roll, pitch, yaw };
        return rpy_array;
    }


    geometry_msgs::Pose transformToWorldFrame(
        const geometry_msgs::Pose& target,
        std::string agv,
        std::string robot) {
        static tf2_ros::StaticTransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        std::string kit_tray;
        if (agv.compare("agv1") == 0)
            kit_tray = "kit_tray_1";
        else if (agv.compare("agv2") == 0)
            kit_tray = "kit_tray_2";
        else if (agv.compare("agv3") == 0)
            kit_tray = "kit_tray_3";
        else if (agv.compare("agv4") == 0)
            kit_tray = "kit_tray_4";

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = kit_tray;
        transformStamped.child_frame_id = "target_frame";
        transformStamped.transform.translation.x = target.position.x;
        transformStamped.transform.translation.y = target.position.y;
        transformStamped.transform.translation.z = target.position.z;
        transformStamped.transform.rotation.x = target.orientation.x;
        transformStamped.transform.rotation.y = target.orientation.y;
        transformStamped.transform.rotation.z = target.orientation.z;
        transformStamped.transform.rotation.w = target.orientation.w;


        for (int i{ 0 }; i < 15; ++i)
            br.sendTransform(transformStamped);

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Rate rate(10);
        ros::Duration timeout(1.0);


        geometry_msgs::TransformStamped world_target_tf;
        geometry_msgs::TransformStamped ee_target_tf;

        std::string ee_link;
        if (robot.compare("gantry") == 0)
            ee_link = "gantry_arm_ee_link";
        else
            ee_link = "ee_link";

        for (int i = 0; i < 10; i++) {
            try {
                world_target_tf = tfBuffer.lookupTransform("world", "target_frame",
                    ros::Time(0), timeout);
            }
            catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

            // this is not being used at the moment
            // try {
            //     ee_target_tf = tfBuffer.lookupTransform("target_frame", ee_link,
            //         ros::Time(0), timeout);
            // }
            // catch (tf2::TransformException& ex) {
            //     ROS_WARN("%s", ex.what());
            //     ros::Duration(1.0).sleep();
            //     continue;
            // }
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
}