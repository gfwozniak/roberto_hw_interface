#include <roberto_hw_interface/roberto_limit_handler.h>

RobertoLimits::RobertoLimits(ros::Publisher * actuator_pub, ros::Publisher * bscrew_pub) 
    : linearActuatorTalon(31),
    ballScrewFalcon(32),
    bscrew_limit_pub(bscrew_pub),
    actuator_limit_pub(actuator_pub)
{
}

bool RobertoLimits::zeroBScrew(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ballScrewFalcon.SetSelectedSensorPosition(0);
    return true;
}

bool RobertoLimits::zeroActuator(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    linearActuatorTalon.SetSelectedSensorPosition(0);
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Roberto_hardware_inerface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);

    auto bscrew_limit_pub = nh.advertise<std_msgs::Bool>("bscrew_limit", 100);
    auto actuator_limit_pub = nh.advertise<std_msgs::Bool>("actuator_limit", 100);
    
    RobertoLimits ROBOT(&bscrew_limit_pub, &actuator_limit_pub);

    ros::ServiceServer zeroBScrewService = nh.advertiseService("zero_bscrew", &RobertoLimits::zeroBScrew, &ROBOT);
    ros::ServiceServer zeroActuatorService = nh.advertiseService("zero_actuator", &RobertoLimits::zeroActuator, &ROBOT);

    spinner.spin();
    
    return 0;
}