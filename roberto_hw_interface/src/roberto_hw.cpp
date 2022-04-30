#include <roberto_hw_interface/roberto_hw.h>

Roberto::Roberto(ros::NodeHandle& nh) 
    : nh_(nh),
    rightDriveFalcon(21), // initialize falcons
    leftDriveFalcon(22),
    linearActuatorTalon(31),
    ballScrewFalcon(32),
    augerFalcon(41)
{

    initRosControlJoints();
    initPhoenixObjects();
    initPositionPublishers();
    
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

void Roberto::initPositionPublishers() {
    bscrew_pos_pub = nh_.advertise<std_msgs::Float64>("bscrew_pos", 1);
    actuator_pos_pub = nh_.advertise<std_msgs::Float64>("actuator_pos", 1);
    hit_limit_switch = nh_.advertise<std_msgs::Bool>("limit_switch", 1);
}

void Roberto::initRosControlJoints() {
        
//// LINEAR ACTUATOR JOINT
//// Create joint_state_interface 
//    hardware_interface::JointStateHandle jointStateHandleActuator("actuator_joint", &actuator_joint_position_, &actuator_joint_velocity_, &actuator_joint_effort_);
//    joint_state_interface_.registerHandle(jointStateHandleActuator);
//// Create position joint interface accepts position command.
//    hardware_interface::JointHandle jointPositionHandleActuator(jointStateHandleActuator, &actuator_joint_position_command_);
//    position_joint_interface_.registerHandle(jointPositionHandleActuator);
//// Create Joint Limit interface 
//    joint_limits_interface::getJointLimits("actuator_joint", nh_, limits);
//    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleActuator(jointPositionHandleActuator, limits);
//    positionJointSaturationInterface.registerHandle(jointLimitsHandleActuator);    

// LINEAR ACTUATOR JOINT (velocity edition)
// Create joint_state_interface 
    hardware_interface::JointStateHandle jointStateHandleActuator("actuator_joint", &actuator_joint_position_, &actuator_joint_velocity_, &actuator_joint_effort_);
    joint_state_interface_.registerHandle(jointStateHandleActuator);
//// Create position joint interface accepts position command.
//    hardware_interface::JointHandle jointVelocityHandleActuator(jointStateHandleActuator, &actuator_joint_position_command_);
//    velocity_joint_interface_.registerHandle(jointVelocityHandleActuator);
// Create position joint interface accepts position command.
    hardware_interface::JointHandle jointPositionHandleActuator(jointStateHandleActuator, &actuator_joint_position_command_);
    position_joint_interface_.registerHandle(jointPositionHandleActuator);

//// BALL SCREW JOINT
//// Create joint_state_interface 
//    hardware_interface::JointStateHandle jointStateHandleBScrew("bscrew_joint", &bscrew_joint_position_, &bscrew_joint_velocity_, &bscrew_joint_effort_);
//    joint_state_interface_.registerHandle(jointStateHandleBScrew);
//// Create position joint interface accepts position command.
//    hardware_interface::JointHandle jointPositionHandleBScrew(jointStateHandleBScrew, &bscrew_joint_position_command_);
//    position_joint_interface_.registerHandle(jointPositionHandleBScrew);
//// Create Joint Limit interface 
//    joint_limits_interface::getJointLimits("bscrew_joint", nh_, limits);
//    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleBScrew(jointPositionHandleBScrew, limits);
//    positionJointSaturationInterface.registerHandle(jointLimitsHandleBScrew);    

// BALL SCREW JOINT (velocity edition)
// Create joint_state_interface 
    hardware_interface::JointStateHandle jointStateHandleBScrew("bscrew_joint", &bscrew_joint_position_, &bscrew_joint_velocity_, &bscrew_joint_effort_);
    joint_state_interface_.registerHandle(jointStateHandleBScrew);
//// Create position joint interface accepts position command.
//    hardware_interface::JointHandle jointVelocityHandleBScrew(jointStateHandleBScrew, &bscrew_joint_position_command_);
//    velocity_joint_interface_.registerHandle(jointVelocityHandleBScrew);
// Create position joint interface accepts position command.
    hardware_interface::JointHandle jointPositionHandleBScrew(jointStateHandleBScrew, &bscrew_joint_position_command_);
    position_joint_interface_.registerHandle(jointPositionHandleBScrew);

// AUGER JOINT
// Create joint state interface
    hardware_interface::JointStateHandle jointStateHandleAuger("auger_joint", &auger_joint_position_, &auger_joint_velocity_, &auger_joint_effort_);
    joint_state_interface_.registerHandle(jointStateHandleAuger);
// Create velocity joint interface
    hardware_interface::JointHandle jointVelocityHandleAuger(jointStateHandleAuger, &auger_joint_velocity_command_);
    velocity_joint_interface_.registerHandle(jointVelocityHandleAuger);


// WHEEL JOINTS 
	for(int i=0; i<2; i++)
	{
	// Create joint state interface
        hardware_interface::JointStateHandle jointStateHandleWheel(wheel_joint_name_[i], &wheel_joint_position_[i], &wheel_joint_velocity_[i], &wheel_joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandleWheel);
    // Create velocity joint interface
	    hardware_interface::JointHandle jointVelocityHandleWheel(jointStateHandleWheel, &wheel_joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandleWheel);
    // Create Joint Limit interface   
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(wheel_joint_name_[i], nh_, limits);
	    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandleWheel(jointVelocityHandleWheel, limits);
	    velocityJointSaturationInterface.registerHandle(jointLimitsHandleWheel);
	}

// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocityJointSaturationInterface);
//    registerInterface(&positionJointSaturationInterface); //Fix limits
    registerInterface(&position_joint_interface_);
//    registerInterface(&effort_joint_interface_);
//    registerInterface(&effortJointSaturationInterface);
}

void Roberto::initPhoenixObjects() 
{
    TalonFXConfiguration wheelConfig;

    ros::param::get("~drive_kP", wheelConfig.slot0.kP);
    ros::param::get("~drive_kI", wheelConfig.slot0.kI);
    ros::param::get("~drive_kD", wheelConfig.slot0.kD);
    ros::param::get("~drive_kF", wheelConfig.slot0.kF);
    ros::param::get("~drive_ramp", wheelConfig.closedloopRamp);
	wheelConfig.slot0.maxIntegralAccumulator = 16000;

	rightDriveFalcon.ConfigAllSettings(wheelConfig);
	leftDriveFalcon.ConfigAllSettings(wheelConfig);

    TalonFXConfiguration bscrewMotionMagic;
    bscrewMotionMagic.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;
    bscrewMotionMagic.motionCruiseVelocity = 15000;
    bscrewMotionMagic.motionAcceleration = 100000;
    bscrewMotionMagic.motionCurveStrength = 1;
    bscrewMotionMagic.slot0.kP = 0.01;
    bscrewMotionMagic.slot0.kI = 0.002;
    bscrewMotionMagic.slot0.maxIntegralAccumulator = 50000;

    ballScrewFalcon.ConfigAllSettings(bscrewMotionMagic);

    TalonSRXConfiguration actuatorMotionMagic;
    actuatorMotionMagic.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonSRXFeedbackDevice::Analog;
    actuatorMotionMagic.motionCruiseVelocity = 1;
    actuatorMotionMagic.motionAcceleration = 100;
    actuatorMotionMagic.motionCurveStrength = 1;
    actuatorMotionMagic.slot0.kP = 50;
    actuatorMotionMagic.slot0.kI = 1;
    actuatorMotionMagic.slot0.maxIntegralAccumulator = 20;

    linearActuatorTalon.ConfigAllSettings(actuatorMotionMagic);
}


//This is the control loop
void Roberto::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}


void Roberto::read() {

// BALL SCREW JOINT READS (FAKE)
    bscrew_joint_position_ = ballScrewFalcon.GetSelectedSensorPosition();
    bscrew_joint_velocity_ = ballScrewFalcon.GetSelectedSensorVelocity();
    bscrew_joint_effort_ = 0;
    // bscrew pos topic
    std_msgs::Float64 bscrew_msg;
    bscrew_msg.data = auger_joint_position_;
    bscrew_pos_pub.publish(bscrew_msg);
    // limit swtich topic
    std_msgs::Bool limit_msg;
    if (ballScrewFalcon.IsFwdLimitSwitchClosed())
    {
        limit_msg.data = 1;
        ROS_INFO("n");
    }
    else
        limit_msg.data = 0;
    hit_limit_switch.publish(limit_msg);

// LINEAR ACTUATOR JOINT READS (FAKE)
    actuator_joint_position_ = 0;
    actuator_joint_velocity_ = 0;
    actuator_joint_effort_ = 0;
    // actuator pos toipc
    std_msgs::Float64 actuator_msg;
    actuator_msg.data = auger_joint_position_;
    actuator_pos_pub.publish(actuator_msg);

// AUGER JOINT READS (FAKE)
    auger_joint_position_ = 0;
    auger_joint_velocity_ = augerFalcon.GetSelectedSensorVelocity();
    auger_joint_effort_ = 0;

// WHEEL JOINT READS
    wheel_joint_position_[0] = rightDriveFalcon.GetSelectedSensorPosition();
    wheel_joint_velocity_[0] = rightDriveFalcon.GetSelectedSensorVelocity();
    wheel_joint_effort_[0] = rightDriveFalcon.GetSupplyCurrent();
    //ROS_INFO("Current Pos: %.2f, Vel: %.2f",wheel_joint_position_[0],wheel_joint_velocity_[0]);
    wheel_joint_position_[1] = leftDriveFalcon.GetSelectedSensorPosition();
    wheel_joint_velocity_[1] = leftDriveFalcon.GetSelectedSensorVelocity();
    wheel_joint_effort_[1] = leftDriveFalcon.GetSupplyCurrent();
}

void Roberto::write(ros::Duration elapsed_time) {
    // Safety
//    velocityJointSaturationInterface.enforceLimits(elapsed_time);   // enforce limits for JointA and JointB
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

    // AUGER WRITES
//    ROS_INFO("Velocity Cmd: %.2f", auger_joint_velocity_command_);

    // WHEEL WRITES
    ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
    rightDriveFalcon.Set(ControlMode::Velocity, wheel_joint_velocity_command_[0]);
    leftDriveFalcon.Set(ControlMode::Velocity, -wheel_joint_velocity_command_[1]);

    // ACTUATOR WRITES
    linearActuatorTalon.Set(ControlMode::MotionMagic, actuator_joint_position_command_);
    ROS_INFO("Actuator Cmd: %.2f",actuator_joint_position_command_);

    // BSCREW WRITES
    ballScrewFalcon.Set(ControlMode::MotionMagic, bscrew_joint_position_command_);
    ROS_INFO("%%Out Cmd: %.2f",bscrew_joint_position_command_);

    // AUGER WRITES
    augerFalcon.Set(ControlMode::PercentOutput, auger_joint_velocity_command_);

    std_msgs::Bool omsg;
    if(ballScrewFalcon.IsFwdLimitSwitchClosed())
    {
        ballScrewFalcon.SetSelectedSensorPosition(0);
        omsg.data = 1;
    }
    else
    {
        omsg.data = 0;
    }
    hit_limit_switch.publish(omsg);
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