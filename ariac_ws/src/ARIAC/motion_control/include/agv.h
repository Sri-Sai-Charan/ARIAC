#ifndef __AGV_H__
#define __AGV_H__

#include <string>
#include <std_msgs/String.h>
#include <nist_gear/AGVToAssemblyStation.h>
#include <ros/ros.h>
#include <ros/console.h>


namespace motioncontrol {
    class Agv {

        public:
        explicit Agv(ros::NodeHandle&);

        /**
         * @brief Submit an AGV shipment
         *
         * @param shipment_type Shipment type which should match the order
         * @param kit_tray  Kit tray used to build the kit
         * @param station Assembly station
         * @return true AGV successfully submitted
         * @return false AGV not successfully submitted
         */
        bool shipAgv(std::string shipment_type, std::string kit_tray, std::string station);
        /**
         * @brief State for AGV1
         *
         * @param msg
         */
        void agv1_state_callback(const std_msgs::String& msg);

        private:
        ros::ServiceClient agv1_client_;
        ros::Subscriber agv1_state_subscriber_;
        bool agv1_ready_;
    };//class
}//namespace

#endif