#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

class Repub 
{
    private:
    ros::Publisher * output;
    double coeff = 1/(8*32767)

    public:
    Repub(ros::Publisher * publisher);
    void callback(const geometry_msgs::Twist::ConstPtr& imsg);
};