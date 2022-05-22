#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <vector>

namespace TelemetryRepublisher
{
    class Republisher 
    {
        private:
        std::vector<ros::Publisher> * output;

        public:
        Republisher(std::vector<ros::Publisher> *);
        void callback(const sensor_msgs::JointState::ConstPtr& imsg);
    };
}