#include <std_srvs/SetBool.h>
#include <std_srvs/SetBool.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace Gantree
{
    class Republisher 
    {
        private:
        ros::Publisher * output;

        public:
        Republisher(ros::Publisher * publisher);
        void callback(const std_msgs::Float64::ConstPtr& imsg);
    };
}
class Gantree_Manager 
{

};