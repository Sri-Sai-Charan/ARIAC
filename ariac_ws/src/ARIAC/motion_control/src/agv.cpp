#include "agv.h"

namespace motioncontrol {
    Agv::Agv(ros::NodeHandle& node)
    {
        agv1_client_ =
            node.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/agv1/submit_shipment");
        // If it's not ready, wait for it to be ready.
        // Calling the Service using the client before the server is ready would fail.
        if (!agv1_client_.exists()) {
            ROS_INFO("Waiting for the AGV1 to be ready...");
            agv1_client_.waitForExistence();
            ROS_INFO("AGV1 is now ready.");
        }

        agv1_state_subscriber_ = node.subscribe(
            "ariac/agv1/state", 10, &Agv::agv1_state_callback, this);
    }


    bool Agv::shipAgv(std::string shipment_type, std::string tray_name, std::string station) {
        nist_gear::AGVToAssemblyStation msg;
        msg.request.assembly_station_name = station;
        msg.request.shipment_type = shipment_type;

        if (tray_name == "kit_tray_1") {
            agv1_client_.call(msg);
        }

        if (msg.response.success) {
            ROS_INFO_STREAM("[agv_control][sendAGV] AGV is taking order: " + msg.request.shipment_type);
            return true;
        }
        else {
            ROS_ERROR_STREAM("[agv_control][sendAGV] Failed to call AGV!");
            return false;
        }
    }

    void Agv::agv1_state_callback(const std_msgs::String& msg)
    {
        if (!((msg.data).compare("ready_to_deliver")))
            agv1_ready_ = true;
        else
            agv1_ready_ = false;
    }
}//namespace
