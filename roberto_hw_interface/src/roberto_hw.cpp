#include <roberto_hw_interface/roberto_hw.h>

Roberto::Roberto(ros::NodeHandle& nh) 
    : nh_(nh),
    talon(2)
     {

// Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();
    
// Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    
//Set the frequency of the control loop.
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    
//Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &Roberto::update, this);
}


Roberto::~Roberto() {
}


void Roberto::init() {
// Phoenix initialize


        
// Create joint_state_interface for JointA
    hardware_interface::JointStateHandle jointStateHandleA("JointA", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandleA);
// Create effort joint interface as JointA accepts effort command.
    hardware_interface::JointHandle jointEffortHandleA(jointStateHandleA, &joint_effort_command_[0]);
    effort_joint_interface_.registerHandle(jointEffortHandleA); 
// Create Joint Limit interface for JointA
    joint_limits_interface::getJointLimits("JointA", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandleA(jointEffortHandleA, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandleA);    

    
// Create joint_state_interface for JointB
    hardware_interface::JointStateHandle jointStateHandleB("JointB", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(jointStateHandleB);

// Create effort joint interface as JointB accepts effort command..
    hardware_interface::JointHandle jointEffortHandleB(jointStateHandleB, &joint_effort_command_[1]);
    effort_joint_interface_.registerHandle(jointEffortHandleB);
// Create Joint Limit interface for JointB
    joint_limits_interface::getJointLimits("JointB", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandleB(jointEffortHandleB, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandleB);    
    



// Create joint_state_interface for JointC
    hardware_interface::JointStateHandle jointStateHandleC("JointC", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
    joint_state_interface_.registerHandle(jointStateHandleC);

// Create position joint interface as JointC accepts position command.
    hardware_interface::JointHandle jointPositionHandleC(jointStateHandleC, &joint_position_command_);
    position_joint_interface_.registerHandle(jointPositionHandleC);
// Create Joint Limit interface for JointC
    joint_limits_interface::getJointLimits("JointC", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleC(jointPositionHandleC, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandleC);    




// JOINT STATE FOR JOINT D
    hardware_interface::JointStateHandle jointStateHandleD("JointD", &joint_position_[4], &joint_velocity_[4], &joint_effort_[4]);
    joint_state_interface_.registerHandle(jointStateHandleD);
// CREATE VELOCITY JOINT INTERFACE AS JOINT D ACCEPTS VELOCITY CMD
    hardware_interface::JointHandle jointVelocityHandleD(jointStateHandleD, &joint_velocity_command_);
    velocity_joint_interface_.registerHandle(jointVelocityHandleD);


// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&effortJointSaturationInterface);
    registerInterface(&positionJointSaturationInterface);    
}


//This is the control loop
void Roberto::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}


void Roberto::read() {
//    ROS_INFO("READING"); 
    for (int i = 0; i < 4; i++)
    {
        joint_position_[i] = 0;
        joint_effort_[i] = 0;
        joint_velocity_[i] = 0;
    }
}

void Roberto::write(ros::Duration elapsed_time) {
    // Safety
    effortJointSaturationInterface.enforceLimits(elapsed_time);   // enforce limits for JointA and JointB
    positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointC

//    ROS_INFO("WRITING");
    std::string s;

    // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
    // the output commands need to send are joint_effort_command_[0] for JointA, joint_effort_command_[1] for JointB and 

    //joint_position_command_ for JointC.

    s = std::to_string(joint_velocity_command_);
    ROS_INFO(s.c_str());

    ctre::phoenix::unmanaged::FeedEnable(100);
    talon.Set(ControlMode::PercentOutput, joint_velocity_command_);

}


int main(int argc, char** argv)
{

    //Initialze the ROS node.
    ros::init(argc, argv, "Roberto_hardware_inerface_node");
    ros::NodeHandle nh;
    
    // Initialize CAN interface with 'can0'
	std::string interface;
	interface = "can0";
	ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    //Separate Spinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(2);
    
    // Create the object of the robot hardware_interface class and spin the thread. 
    Roberto ROBOT(nh);
    spinner.spin();
    
    return 0;
}