// ###################################################################
// ################### Importing Necessary Libraries #################
// ###################################################################

#include <vector>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <std_srvs/Trigger.h>

// ###################################################################


/**
 * @brief A Class called Main handler which handles starting the competition and getting competition state 
 * 
 */
class MainHandler
{   
    private:
    ros::NodeHandle nh;
    ros::Subscriber competion_state_sub;
    ros::ServiceClient competition_start_client;

    public:
    
    /**
     * @brief Construct a new Main Handler object
     * 
     * @param node_handler 
     */
    MainHandler(ros::NodeHandle & node_handler);

    /**
     * @brief init function
     * 
     */
    void init();


    /**
     * @brief getting the compition state
     * 
     * @param msg 
     */
    void competition_state_callback(const std_msgs::String::ConstPtr& msg);



    /**
     * @brief To start the compition if the state exisits, if not it will return failed msg
     * 
     * @return true 
     * @return false 
     */
    bool start_competition();
};