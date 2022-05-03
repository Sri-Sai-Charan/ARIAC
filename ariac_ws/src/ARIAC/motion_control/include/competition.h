#ifndef __COMPETITION_H__
#define __COMPETITION_H__

#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>

namespace motioncontrol {
    /**
     * @brief Competition class
     *
     */
    class Competition
    {
        public:
        /**
         * @brief Construct a new Competition object
         *
         * @param node Node handle
         */
        explicit Competition(ros::NodeHandle& node);
        /**
         * @brief Initialize components of the class (suscribers, publishers, etc)
         *
         */
        void init();
        /**
         * @brief Start the competition through ROS service
         *
         */
        void startCompetition();
        /**
         * @brief End competition through ROS service
         *
         */
        void endCompetition();
        /**
         * @brief Get the state of the competition (init, end, etc)
         *
         * @param msg
         */
        void competition_state_callback(const std_msgs::String::ConstPtr& msg);
        /**
         * @brief Time since the competition started
         *
         * @param msg
         */
        void competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg);
        /**
         * @brief Get the Clock objectGet
         *
         * @return double
         */
        double getClock();
        /**
         * @brief Get the Start Time object
         *
         * @return double
         */
        double getStartTime();
        /**
         * @brief Get the Competition State object
         *
         * @return std::string
         */
        std::string getCompetitionState();


        private:
        /*!< node handle for this class */
        ros::NodeHandle node_;
        /*!< state of the competition */
        std::string competition_state_;
        /*!< current score during the trial */
        double current_score_;
        /*!< clock to check if we are close to the time limit */
        ros::Time competition_clock_;
        /*!< wall time in second */
        double competition_start_time_;
        /*!< subscriber to the topic /ariac/current_score */
        ros::Subscriber current_score_subscriber_;
        /*!< subscriber to the topic /ariac/competition_state */
        ros::Subscriber competition_state_subscriber_;
        /*!< subscriber to the topic /clock */
        ros::Subscriber competition_clock_subscriber_;
        /*!< subscriber to the topic /ariac/orders */
        ros::Subscriber orders_subscriber_;
    };
}//namespace

#endif