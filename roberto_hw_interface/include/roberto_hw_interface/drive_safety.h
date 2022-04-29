#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

namespace DriveSafety
{
    class Republisher 
    {
        private:
        ros::Publisher * outputTwist;
        double actuator_max;
        double actuator_min;
        double bscrew_max;
        double bscrew_min;
        double bscrew_position = 0;
        double actuator_position = 0;
        geometry_msgs::Twist zeroOutput;
        bool isDriveable();

        public:
        Republisher(ros::Publisher * publisher);
        void callback(const geometry_msgs::Twist::ConstPtr& imsg);
        void callbackActuatorPosition(const std_msgs::Float64::ConstPtr& imsg);
        void callbackBScrewPosition(const std_msgs::Float64::ConstPtr& imsg);
    };
}