//ros
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
//moveit
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
//standard library
#include <math.h>
//custom
#include "competition.h"
#include "gantry.h"
#include "arm.h"
#include "agv.h"
#include "utils.h"
#include "part.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_cpp_node");
    ros::NodeHandle node_handle;

    // 0 means that the spinner will use as many threads as there are processors on your machine.
    //If you use 3 for example, only 3 threads will be used.
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // start the competition
    motioncontrol::Competition competition(node_handle);
    competition.init();
    std::string competition_state = competition.getCompetitionState();
    auto competition_start_time = competition.getClock();

    //pickandplace
    std::string demo = "pickandplace";
    //kitting or assembly
    std::string task = "kitting";
    //gantry or arm
    std::string robot = "gantry";

    bool visual = true;

    if (visual) {
        // Visualization
        // ^^^^^^^^^^^^^
        // https://github.com/ros-planning/moveit_tutorials/blob/melodic-devel/doc/planning_scene_ros_api/src/planning_scene_ros_api_tutorial.cpp
        // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
        // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
        moveit_visual_tools::MoveItVisualTools visual_tools("torso_base_main_joint");
        visual_tools.deleteAllMarkers();
        // ROS API
        // ^^^^^^^
        // The ROS API to the planning scene publisher is through a topic interface
        // using "diffs". A planning scene diff is the difference between the current
        // planning scene (maintained by the move_group node) and the new planning
        // scene desired by the user.
        //
        // Advertise the required topic
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // We create a publisher and wait for subscribers
        // Note that this topic may need to be remapped in the launch file
        ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/ariac/gantry/planning_scene", 1);
        ros::WallDuration sleep_t(0.5);
        while (planning_scene_diff_publisher.getNumSubscribers() < 1) {
            sleep_t.sleep();
        }
        
        // Define the attached object message
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // We will use this message to add or
        // subtract the object from the world
        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = "world";
        /* The header must contain a valid TF frame*/
        attached_object.object.header.frame_id = "world";
        /* The id of the object */
        attached_object.object.id = "box";

        /* A default pose */
        geometry_msgs::Pose pose;
        pose.position.x = -6.0;
        pose.position.z = 2.0;
        pose.orientation.w = 1.0;

        /* Define a box to be attached */
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[0] = 0.2;
        // primitive.dimensions[1] = 0.4;
        // primitive.dimensions[2] = 2;

        attached_object.object.primitives.push_back(primitive);
        attached_object.object.primitive_poses.push_back(pose);

        // Note that attaching an object to the robot requires
        // the corresponding operation to be specified as an ADD operation
        attached_object.object.operation = attached_object.object.ADD;

        // Since we are attaching the object to the robot hand to simulate picking up the object,
        // we want the collision checker to ignore collisions between the object and the robot hand
        attached_object.touch_links = std::vector<std::string>{ };

        // Add an object into the environment
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Add the object into the environment by adding it to
        // the set of collision objects in the "world" part of the
        // planning scene. Note that we are using only the "object"
        // field of the attached_object message here.
        ROS_INFO("Adding the object into the world.");
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(attached_object.object);
        planning_scene.is_diff = true;
        planning_scene_diff_publisher.publish(planning_scene);
        ros::Duration(20.0).sleep();
    }
    if (demo == "pickandplace") {
        // Set up the part we want the robot to pick up
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Build a part object from the class Part
        // - Store the model: This should be retrieved from /ariac/orders
        // - Store the frame for this part: This is optional in this example
        // - Store the initial pose of the part in the camera frame
        //   - This should be retrieved in the callback of the camera
        // - Store the initial pose in the world frame
        //    - This should be done with a lookup transform
        // - Store the target pose represented in the tray frame
        //    - This should be retrieved from /ariac/orders
        // - Store the frame of the tray where this part should be placed
        //    - We need this information for the broadcaster
        // - Store the agv used for this task
        //    - We need this to ship the correct AGV
        auto part_model{ "assembly_pump_red" };
        auto camera_frame{ "logical_camera_bin1234_assembly_pump_red_1_frame" };
        motioncontrol::Part part{ part_model ,camera_frame };
        //init pose (in bin)  of the part in the camera frame
        //get this in the camera callback
        part.setInitPoseInFrame();
        //init pose (in bin) of the part in the world frame
        //transform from camera frame to world frame
        part.setInitPoseInWorld(0, 0, 0);
        // target pose (in tray) of the part in tray frame
        // get this from /ariac/orders
        part.setTargetPoseInFrame(0.1, 0.1, 0.1, 0, 0, M_PI / 3);
        // frame name where to place the part
        // get this from /ariac/orders
        part.setTargetFrame("kit_tray_1");
        // agv where to place the part
        // get this from /ariac/orders
        part.setAGV("agv1");

        // Are we doing kitting or assembly?
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        if (task.compare("kitting") == 0) {
            // Using the linear arm for kitting
            // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            if (robot.compare("arm") == 0) {
                // initialize the UR10 arm
                motioncontrol::Arm arm(node_handle);
                arm.init();
                arm.goToPresetLocation(arm.home_);
                arm.goToPresetLocation(arm.at_bin1_);
                if (arm.pickPart(part)) {
                    arm.goToPresetLocation(arm.at_agv1_);
                    if (arm.placePart(part)) {
                        // ship agv
                        motioncontrol::Agv agv_control(node_handle);
                        // TODO: The following arguments should be retrieved from /ariac/orders
                        agv_control.shipAgv("order_0_kitting_shipment_0", "kit_tray_1", "as2");

                        // end the competition
                        competition.endCompetition();
                    }
                }
            }
            // Using the gantry for kitting
            // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            else if (robot.compare("gantry") == 0) {
                motioncontrol::Gantry gantry(node_handle);
                gantry.init();
                gantry.goToPresetLocation(gantry.home_);
                gantry.goToPresetLocation(gantry.safe_bins_);
                gantry.goToPresetLocation(gantry.at_bins1234_);
                gantry.goToPresetLocation(gantry.at_bin1_);
                if (gantry.pickPart(part)) {
                    ROS_INFO_STREAM("Part picked up");
                }
                else {
                    ros::shutdown();
                }
                if (gantry.placePart(part)) {
                    ROS_INFO_STREAM("Part placed");

                    gantry.goToPresetLocation(gantry.at_bin1_);
                    gantry.goToPresetLocation(gantry.at_bins1234_);
                    gantry.goToPresetLocation(gantry.safe_bins_);
                    gantry.goToPresetLocation(gantry.home_);
                    // ship agv
                    motioncontrol::Agv agv_control(node_handle);
                    // TODO: The following arguments should be retrieved from /ariac/orders
                    agv_control.shipAgv("order_0_kitting_shipment_0", "kit_tray_1", "as2");

                    // end the competition
                    competition.endCompetition();
                }

            }
        }
        else if (task.compare("assembly") == 0) {
            // TODO: implement assembly task with the gantry only
            // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        }
    }

    ros::waitForShutdown();
}