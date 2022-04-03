#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

namespace TopicConvert
{
    class Republisher 
    {
        private:
        ros::Publisher * output;

        public:
        Republisher(ros::Publisher * publisher);
        void callback(const geometry_msgs::Twist::ConstPtr& imsg);
    };
}