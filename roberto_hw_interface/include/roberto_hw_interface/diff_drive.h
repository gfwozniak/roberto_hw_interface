#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

class Repub 
{
    private:
    ros::Publisher * outputLeft;
    ros::Publisher * outputRight;
    double totalMod;
    double leftMod;
    double rightMod;
    double turnMod;

    public:
    Repub(ros::Publisher * publisherRight, ros::Publisher * publisherLeft);
    void callback(const geometry_msgs::Twist::ConstPtr& imsg);
};