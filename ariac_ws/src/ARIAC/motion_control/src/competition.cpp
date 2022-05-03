#include "competition.h"
#include <std_srvs/Trigger.h>


namespace motioncontrol {
    ////////////////////////
    Competition::Competition(ros::NodeHandle& node) : current_score_(0) {
        node_ = node;
    }

    ////////////////////////
    void Competition::init() {
        double time_called = ros::Time::now().toSec();
        competition_start_time_ = ros::Time::now().toSec();

        // subscribe to the '/ariac/competition_state' topic.
        competition_state_subscriber_ = node_.subscribe(
            "/ariac/competition_state", 10, &Competition::competition_state_callback, this);

        // subscribe to the '/clock' topic.
        competition_clock_subscriber_ = node_.subscribe(
            "/clock", 10, &Competition::competition_clock_callback, this);

        // start the competition
        startCompetition();
    }

    ////////////////////////
    void Competition::competition_state_callback(const std_msgs::String::ConstPtr& msg) {
        if (msg->data == "done" && competition_state_ != "done") {
            ROS_INFO("Competition ended.");
        }
        competition_state_ = msg->data;
    }


    ////////////////////////
    void Competition::competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg) {
        competition_clock_ = msg->clock;
    }

    ////////////////////////
    void Competition::startCompetition() {
        // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
        ros::ServiceClient start_client =
            node_.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
        // If it's not already ready, wait for it to be ready.
        // Calling the Service using the client before the server is ready would fail.
        if (!start_client.exists()) {
            start_client.waitForExistence();
        }
        std_srvs::Trigger srv;
        start_client.call(srv);
        if (!srv.response.success) {  // If not successful, print out why.
            ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
        }
        else {
            ROS_INFO("Competition started!");
        }
    }

    ////////////////////////
    void Competition::endCompetition() {
        // Create a Service client for the correct service, i.e. '/ariac/end_competition'.
        ros::ServiceClient end_client =
            node_.serviceClient<std_srvs::Trigger>("/ariac/end_competition");
        // If it's not already ready, wait for it to be ready.
        // Calling the Service using the client before the server is ready would fail.
        if (!end_client.exists()) {
            end_client.waitForExistence();
        }
        std_srvs::Trigger srv;
        end_client.call(srv);
        if (!srv.response.success) {  // If not successful, print out why.
            ROS_ERROR_STREAM("Failed to end the competition: " << srv.response.message);
        }
        else {
            ROS_INFO("Competition ended!");
        }
    }

    ////////////////////////
    double Competition::getStartTime() {
        return competition_start_time_;
    }

    ////////////////////////
    double Competition::getClock() {
        double time_spent = competition_clock_.toSec();
        return time_spent;
    }

    ////////////////////////
    std::string Competition::getCompetitionState() {
        return competition_state_;
    }
}
