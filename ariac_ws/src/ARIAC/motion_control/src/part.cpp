#include "part.h"
#include "utils.h"
//ros
#include <tf2/LinearMath/Quaternion.h>

namespace motioncontrol {
    Part::Part(ros::NodeHandle& node) {
        node_ = node;
    }

    Part::Part(std::string model, std::string init_frame) :
        model_{ model },
        init_frame_{ init_frame } {
    }

    void Part::setInitFrame(std::string init_frame) {
        this->init_frame_ = init_frame;
    }

    void Part::setInitPoseInFrame() {
        geometry_msgs::Pose init_pose_in_frame;
        init_pose_in_frame.position.x = 1.02005077798;
        init_pose_in_frame.position.y = -0.143784743476;
        init_pose_in_frame.position.z = -0.285406125404;
        init_pose_in_frame.orientation.x = -0.710729633761;
        init_pose_in_frame.orientation.y = -0.0103267017871;
        init_pose_in_frame.orientation.z = 0.703388294809;
        init_pose_in_frame.orientation.w = -0.00128594283435;
        this->init_pose_in_frame_ = init_pose_in_frame;
    }

    void Part::setInitPoseInWorld(double r, double p, double y) {
        geometry_msgs::Pose init_pose_in_world;
        init_pose_in_world.position.x = -1.998993;
        init_pose_in_world.position.y = 3.279920;
        init_pose_in_world.position.z = 0.779616;
        // create this quaternion from roll/pitch/yaw (in radians)
        // note: you will directly get the orientation in quaternion from a lookup transform
        // you do not need to convert from Euler to Quaternion
        tf2::Quaternion part_world_frame_q;
        part_world_frame_q.setRPY(r, p, y);
        init_pose_in_world.orientation.x = part_world_frame_q.getX();
        init_pose_in_world.orientation.y = part_world_frame_q.getY();
        init_pose_in_world.orientation.z = part_world_frame_q.getZ();
        init_pose_in_world.orientation.w = part_world_frame_q.getW();
        this->init_pose_in_world_ = init_pose_in_world; 
    }

    void Part::setAGV(std::string agv) {
        this->agv_ = agv;
    }
    std::string Part::getAGV() {
        return this->agv_;
    }
    geometry_msgs::Pose Part::getInitPoseInWorld() {
        return this->init_pose_in_world_;
    }

    geometry_msgs::Pose Part::getTargetPoseInFrame() {
        return this->target_pose_in_frame_;
    }

    void Part::setModel(std::string model) {
        this->model_ = model;
    }

    void Part::setTargetFrame(std::string target_frame) {
        this->target_frame_ = target_frame;
    }
    
    void Part::setTargetPoseInFrame(
        double x,
        double  y,
        double  z,
        double roll,
        double  pitch,
        double  yaw) {

        geometry_msgs::Pose target_pose_in_frame;
        target_pose_in_frame.position.x = x;
        target_pose_in_frame.position.y = y;
        target_pose_in_frame.position.z = z;
        auto q = motioncontrol::quaternionFromEuler(roll, pitch, yaw);
        target_pose_in_frame.orientation.x = q.getX();
        target_pose_in_frame.orientation.y = q.getY();
        target_pose_in_frame.orientation.z = q.getZ();
        target_pose_in_frame.orientation.w = q.getW();
        this->target_pose_in_frame_ = target_pose_in_frame;
        
    }

    void Part::setTargetPoseInWorld(const geometry_msgs::Pose& target_pose_in_world) {
        this->target_pose_in_world_ = target_pose_in_world;
    }
}
