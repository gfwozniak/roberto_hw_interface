#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

namespace DiffDrive
{
    class Republisher 
    {
        private:
        ros::Publisher * outputLeft;
        ros::Publisher * outputRight;
        double totalMod;
        double leftMod;
        double rightMod;
        double turnMod;

        public:
        Republisher(ros::Publisher * publisherRight, ros::Publisher * publisherLeft);
        void callback(const geometry_msgs::Twist::ConstPtr& imsg);
    };
}