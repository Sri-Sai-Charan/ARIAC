#ifndef __PART_H__
#define __PART_H__


//ros
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
//standard library
#include <string>


namespace motioncontrol {
    /**
     * @brief Part class
     *
     * Store information about a part.
     *
     */
    class Part
    {
        public:
        /**
         * @brief Construct a new Part object
         *
         * @param node Node handle
         */
        explicit Part(ros::NodeHandle& node);
        Part(std::string model, std::string init_frame);
        //!@brief Set the attribute init_pose_in_frame
        void setInitPoseInFrame();
        //!@brief Set the attribute init_pose_in_world
        void setInitPoseInWorld(double r, double p, double y);
        //!@brief Set the attribute target_pose_in_frame
        void setTargetPoseInFrame(double x, double  y, double  z, double roll, double  pitch, double  yaw);
        //! @brief Set the attribute target_pose_in_world
        void setTargetPoseInWorld(const geometry_msgs::Pose& target_pose_in_world);
        //! @brief Set the model_ attribute
        void setModel(std::string model);
        //! @brief Set the init_frame_ attribute
        void setInitFrame(std::string init_frame);

        void setTargetFrame(std::string target_frame);
        void setAGV(std::string agv);
        std::string getAGV();

        geometry_msgs::Pose getInitPoseInWorld();
        geometry_msgs::Pose getTargetPoseInFrame();


        private:
        /*!< node handle for this class */
        ros::NodeHandle node_;
        /*!< model of the part (e.g., assembly_battery_blue)*/
        std::string model_;
        /*!< frame in which the part is detected (e.g., "LC_BIN_SET1_assembly_pump_red_1_frame")*/
        std::string init_frame_;
        /*!< initial part pose in camera or tray frame*/
        geometry_msgs::Pose init_pose_in_frame_;
        /*!< initial part pose in world frame*/
        geometry_msgs::Pose init_pose_in_world_; // init pose in world frame
        // frame that will be created by a broadcaster
        std::string target_frame_;
        /*!< target part pose in tray frame (from /ariac/orders)*/
        geometry_msgs::Pose target_pose_in_frame_;
        /*!< target part pose in world frame (lookup transform from target_frame to "world")*/
        geometry_msgs::Pose target_pose_in_world_;
        /*!< whether the part is faulty or not */
        bool faulty_;
        /*!< agv on which this part should be placed */
        std::string agv_;
    };// end class
}// end namespace
#endif