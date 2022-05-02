#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>

// UPDATED PHOENIX INCLUDE
#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <string>
#include <thread>
#include <unistd.h>

class RobertoLimits
{
    public:
        RobertoLimits(ros::Publisher * actuator_pub, ros::Publisher * bscrew_pub);
        bool zeroBScrew(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        bool zeroActuator(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        void enableFeed(const sensor_msgs::JointState::ConstPtr& imsg);
        void readLimit(const ros::TimerEvent& timer);
        
    protected:
        ctre::phoenix::motorcontrol::can::TalonSRX linearActuatorTalon;
        ctre::phoenix::motorcontrol::can::TalonFX ballScrewFalcon;

        ros::Publisher * bscrew_limit_pub;
        ros::Publisher * actuator_limit_pub;

};