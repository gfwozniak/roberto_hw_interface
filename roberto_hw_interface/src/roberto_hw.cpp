#include <roberto_hw_interface/roberto_hw.h>

Roberto::Roberto(ros::NodeHandle& nh) 
    : nh_(nh),
    rightDriveTalon(21, interface), // initialize falcons
    leftDriveTalon(22, interface)
{

// Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    initJoints();
    initDrive();
    
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


void Roberto::initJoints() {
        
//// Create joint_state_interface for JointA
//    hardware_interface::JointStateHandle jointStateHandleA("JointA", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
//    joint_state_interface_.registerHandle(jointStateHandleA);
//// Create effort joint interface as JointA accepts effort command.
//    hardware_interface::JointHandle jointEffortHandleA(jointStateHandleA, &joint_effort_command_[0]);
//    effort_joint_interface_.registerHandle(jointEffortHandleA); 
//// Create Joint Limit interface for JointA
//    joint_limits_interface::getJointLimits("JointA", nh_, limits);
//    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandleA(jointEffortHandleA, limits);
//    effortJointSaturationInterface.registerHandle(jointLimitsHandleA);    
//
//    
//// Create joint_state_interface for JointB
//    hardware_interface::JointStateHandle jointStateHandleB("JointB", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
//    joint_state_interface_.registerHandle(jointStateHandleB);
//
//// Create effort joint interface as JointB accepts effort command..
//    hardware_interface::JointHandle jointEffortHandleB(jointStateHandleB, &joint_effort_command_[1]);
//    effort_joint_interface_.registerHandle(jointEffortHandleB);
//// Create Joint Limit interface for JointB
//    joint_limits_interface::getJointLimits("JointB", nh_, limits);
//    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandleB(jointEffortHandleB, limits);
//    effortJointSaturationInterface.registerHandle(jointLimitsHandleB);    
//    
//
//
//
//// Create joint_state_interface for JointC
//    hardware_interface::JointStateHandle jointStateHandleC("JointC", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
//    joint_state_interface_.registerHandle(jointStateHandleC);
//
//// Create position joint interface as JointC accepts position command.
//    hardware_interface::JointHandle jointPositionHandleC(jointStateHandleC, &joint_position_command_);
//    position_joint_interface_.registerHandle(jointPositionHandleC);
//// Create Joint Limit interface for JointC
//    joint_limits_interface::getJointLimits("JointC", nh_, limits);
//    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleC(jointPositionHandleC, limits);
//    positionJointSaturationInterface.registerHandle(jointLimitsHandleC);    

	for(int i=0; i<2; i++)
	{
	// Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);
       
    // Create velocity joint interface
	    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);

    // Create Joint Limit interface   
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(joint_name_[i], nh_, limits);
	    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
	    velocityJointSaturationInterface.registerHandle(jointLimitsHandle);

	}

// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocityJointSaturationInterface);
//    registerInterface(&effort_joint_interface_);
//    registerInterface(&position_joint_interface_);
//    registerInterface(&effortJointSaturationInterface);
//    registerInterface(&positionJointSaturationInterface);
}

void Roberto::initDrive() 
{
    TalonFXConfiguration config;

    ros::param::get("~drive_kP", config.slot0.kP);
    ros::param::get("~drive_kI", config.slot0.kI);
    ros::param::get("~drive_kD", config.slot0.kD);
    ros::param::get("~drive_kF", config.slot0.kF);
    ros::param::get("~drive_ramp", config.closedloopRamp);
	config.slot0.maxIntegralAccumulator = 16000;

	rightDriveTalon.ConfigAllSettings(config);
	leftDriveTalon.ConfigAllSettings(config);
}


//This is the control loop
void Roberto::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}


void Roberto::read() {
// Joint R read
    joint_position_[0] = rightDriveTalon.GetSelectedSensorPosition();
    joint_velocity_[0] = rightDriveTalon.GetSelectedSensorVelocity();
    joint_effort_[0] = rightDriveTalon.GetSupplyCurrent();
    ROS_INFO("Current Pos: %.2f, Vel: %.2f",joint_position_[0],joint_velocity_[0]);
// Joint L read
    joint_position_[1] = leftDriveTalon.GetSelectedSensorPosition();
    joint_velocity_[1] = leftDriveTalon.GetSelectedSensorVelocity();
    joint_effort_[1] = leftDriveTalon.GetSupplyCurrent();
//    ROS_INFO("Current Pos: %.2f, Vel: %.2f",joint_position_[1],joint_velocity_[1]);
}

void Roberto::write(ros::Duration elapsed_time) {
    // Safety
    velocityJointSaturationInterface.enforceLimits(elapsed_time);   // enforce limits for JointA and JointB
//    positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointC

    // Right motor control
//    std::string s1;
//
//    s1 = std::to_string(joint_velocity_command_[0]);
//    ROS_INFO("Joint D");
//    ROS_INFO(s1.c_str());

//    ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
//    rightDriveTalon.Set(ControlMode::PercentOutput, joint_velocity_command_[0]);

    // Left motor control
//    std::string s2;
//
//    s2 = std::to_string(joint_velocity_command_[1]);
//    ROS_INFO("Joint E");
//    ROS_INFO(s2.c_str());

//    leftDriveTalon.Set(ControlMode::PercentOutput, joint_velocity_command_[1]);

    ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
    rightDriveTalon.Set(ControlMode::PercentOutput, joint_velocity_command_[0]);
    ROS_INFO("%%Out Cmd: %.2f",joint_velocity_command_[0]);
    leftDriveTalon.Set(ControlMode::PercentOutput, joint_velocity_command_[1]);
//    ROS_INFO("%%Out Cmd: %.2f",joint_effort_command_[0]);
}


int main(int argc, char** argv)
{

    //Initialze the ROS node.
    ros::init(argc, argv, "Roberto_hardware_inerface_node");
    ros::NodeHandle nh;
    
    //Separate Spinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(2);
    
    // Create the object of the robot hardware_interface class and spin the thread. 
    Roberto ROBOT(nh);
    spinner.spin();
    
    return 0;
}