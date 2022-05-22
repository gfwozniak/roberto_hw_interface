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

void Roberto::initRosControlJoints() {
        
// LINEAR ACTUATOR JOINT
    hardware_interface::JointStateHandle jointStateHandleActuator("actuator_joint", &actuator_joint_position_, &actuator_joint_velocity_, &actuator_joint_effort_);
    joint_state_interface_.registerHandle(jointStateHandleActuator);
    hardware_interface::JointHandle jointPositionHandleActuator(jointStateHandleActuator, &actuator_joint_position_command_);
    position_joint_interface_.registerHandle(jointPositionHandleActuator);

// BALL SCREW JOINT 
    hardware_interface::JointStateHandle jointStateHandleBScrew("bscrew_joint", &bscrew_joint_position_, &bscrew_joint_velocity_, &bscrew_joint_effort_);
    joint_state_interface_.registerHandle(jointStateHandleBScrew);
    hardware_interface::JointHandle jointPositionHandleBScrew(jointStateHandleBScrew, &bscrew_joint_position_command_);
    position_joint_interface_.registerHandle(jointPositionHandleBScrew);

// AUGER JOINT
    hardware_interface::JointStateHandle jointStateHandleAuger("auger_joint", &auger_joint_position_, &auger_joint_velocity_, &auger_joint_effort_);
    joint_state_interface_.registerHandle(jointStateHandleAuger);
    hardware_interface::JointHandle jointVelocityHandleAuger(jointStateHandleAuger, &auger_joint_velocity_command_);
    velocity_joint_interface_.registerHandle(jointVelocityHandleAuger);

// FAKE LIMIT SWITCH JOINT
    hardware_interface::JointStateHandle jointStateHandleLimitSwitch("limit_switch", &limit_switch_position_, &limit_switch_velocity_, &limit_switch_effort_);
    joint_state_interface_.registerHandle(jointStateHandleLimitSwitch);
    hardware_interface::JointHandle jointVelocityHandleLimitSwitch(jointStateHandleLimitSwitch, &limit_switch_zero_);
    velocity_joint_interface_.registerHandle(jointVelocityHandleLimitSwitch);

// WHEEL JOINTS 
	for(int i=0; i<2; i++)
	{
        hardware_interface::JointStateHandle jointStateHandleWheel(wheel_joint_name_[i], &wheel_joint_position_[i], &wheel_joint_velocity_[i], &wheel_joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandleWheel);
	    hardware_interface::JointHandle jointVelocityHandleWheel(jointStateHandleWheel, &wheel_joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandleWheel);
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(wheel_joint_name_[i], nh_, limits);
	    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandleWheel(jointVelocityHandleWheel, limits);
	    velocityJointSaturationInterface.registerHandle(jointLimitsHandleWheel);
	}

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocityJointSaturationInterface);
    registerInterface(&position_joint_interface_);
}

void Roberto::initPhoenixObjects() 
{
    TalonFXConfiguration wheelConfig;
    wheelConfig.slot0.kP = 0.1;
    wheelConfig.slot0.kI = 0.002;
    wheelConfig.slot0.kD = 5.0;
    wheelConfig.slot0.kF = 0.035;
    wheelConfig.closedloopRamp = 1;
	wheelConfig.slot0.maxIntegralAccumulator = 16000;
	rightDriveFalcon.ConfigAllSettings(wheelConfig);
    rightDriveFalcon.SetInverted(true);
	leftDriveFalcon.ConfigAllSettings(wheelConfig);

    TalonFXConfiguration bscrewMotionMagic;
    bscrewMotionMagic.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;
    bscrewMotionMagic.motionCruiseVelocity = 25000;
    bscrewMotionMagic.motionAcceleration = 100000;
    bscrewMotionMagic.motionCurveStrength = 0;
    bscrewMotionMagic.slot0.kP = 0.005;
    bscrewMotionMagic.clearPositionOnLimitF = true;
//    bscrewMotionMagic.slot0.kI = 0.0002;
//    bscrewMotionMagic.slot0.maxIntegralAccumulator = 50000;
    ballScrewFalcon.ConfigAllSettings(bscrewMotionMagic);

    TalonSRXConfiguration actuatorMotionMagic;
    actuatorMotionMagic.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonSRXFeedbackDevice::Analog;
    actuatorMotionMagic.motionCruiseVelocity = 2;
    actuatorMotionMagic.motionAcceleration = 5;
    actuatorMotionMagic.motionCurveStrength = 0;
    actuatorMotionMagic.slot0.kP = 30;
    linearActuatorTalon.ConfigAllSettings(actuatorMotionMagic);

    ros::param::get("~wheel_multiplier", wheelMultiplier);
    ros::param::get("~current_threshold", currentThreshold);
}


//This is the control loop
void Roberto::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}


void Roberto::read() {

    // WHEEL READS
    wheel_joint_position_[1] = rightDriveFalcon.GetSelectedSensorPosition();
    wheel_joint_velocity_[1] = rightDriveFalcon.GetSelectedSensorVelocity();
    wheel_joint_effort_[1] = rightDriveFalcon.GetSupplyCurrent();
    wheel_joint_position_[0] = leftDriveFalcon.GetSelectedSensorPosition();
    wheel_joint_velocity_[0] = leftDriveFalcon.GetSelectedSensorVelocity();
    wheel_joint_effort_[0] = leftDriveFalcon.GetSupplyCurrent();

    // BSCREW READS
    bscrew_joint_position_ = ballScrewFalcon.GetSelectedSensorPosition();
    bscrew_joint_velocity_ = ballScrewFalcon.GetSelectedSensorVelocity();
    bscrew_joint_effort_ = 0;

    // ACTUATOR READS
    actuator_joint_position_ = linearActuatorTalon.GetSelectedSensorPosition();
    actuator_joint_velocity_ = linearActuatorTalon.GetSelectedSensorVelocity();
    actuator_joint_effort_ = linearActuatorTalon.GetBusVoltage();

    // AUGER READS
    auger_joint_position_ = augerFalcon.GetSupplyCurrent();
    auger_joint_velocity_ = augerFalcon.GetSelectedSensorVelocity();
    auger_joint_effort_ = augerFalcon.GetTemperature();

    // LIMIT READS
    limit_switch_position_ = ballScrewFalcon.IsFwdLimitSwitchClosed();
//    ballScrewFalcon.ConfigClearPositionOnLimitF(false, 2000);
    limit_switch_velocity_ = 0;
    limit_switch_effort_ = 100;

}

void Roberto::write(ros::Duration elapsed_time) {
//    debugging 
//    ROS_INFO("Value: %.2f", variable);
    ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);

    double apos = linearActuatorTalon.GetSelectedSensorPosition();
    double bpos = ballScrewFalcon.GetSelectedSensorPosition();
    bool is_actuator_neutral = (apos > -20);
    bool is_bscrew_neutral = (bpos > -20000);
    bool is_actuator_mine = (apos < -70);

    // WHEEL WRITES
    if (is_actuator_neutral && is_bscrew_neutral)
    {
        rightDriveFalcon.Set(ControlMode::Velocity, (wheel_joint_velocity_command_[1] * wheelMultiplier));
        leftDriveFalcon.Set(ControlMode::Velocity, (wheel_joint_velocity_command_[0] * wheelMultiplier));
    }
    else
    {
        rightDriveFalcon.Set(ControlMode::PercentOutput, 0);
        leftDriveFalcon.Set(ControlMode::PercentOutput, 0);
    }

    // ACTUATOR WRITES
    double amax = actuator_joint_position_command_ + 1;
    double amin = actuator_joint_position_command_ - 1;
    if (apos < amax && apos > amin)
    {
        linearActuatorTalon.Set(ControlMode::PercentOutput, 0);
    }
    else
    {
        linearActuatorTalon.Set(ControlMode::MotionMagic, actuator_joint_position_command_);
    }

    // BSCREW WRITES
    double bmax = bscrew_joint_position_command_ + 5000;
    double bmin = bscrew_joint_position_command_ - 5000;
    if (ballScrewFalcon.IsFwdLimitSwitchClosed())
    {
        ballScrewFalcon.Set(ControlMode::PercentOutput, -.9);
    }
    else if (bpos < bmax && bpos > bmin)
    {
        ballScrewFalcon.Set(ControlMode::PercentOutput, 0);
    }
    else
    {
        ballScrewFalcon.Set(ControlMode::MotionMagic, bscrew_joint_position_command_);
    }
    
    // AUGER WRITES
    double current = augerFalcon.GetStatorCurrent();
    if (current < currentThreshold)
    {
        augerFalcon.Set(ControlMode::PercentOutput, auger_joint_velocity_command_);
    }
    else
    {
        augerFalcon.Set(ControlMode::PercentOutput, (-0.8 * auger_joint_velocity_command_));
    }

    // LIMITS
    if (limit_switch_zero_ > 1)
    {
        linearActuatorTalon.SetSelectedSensorPosition(0);
    }
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