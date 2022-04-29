#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

// UPDATED PHOENIX INCLUDE
#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <string>
#include <thread>
#include <unistd.h>

class Roberto : public hardware_interface::RobotHW 
{
    public:
        Roberto(ros::NodeHandle& nh);
        ~Roberto();
        void initRosControlJoints(); // Initializes all ros_control joints
        void initPhoenixObjects(); // Configures all falcon motors
        void initPositionPublishers(); // Advertises position states
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        
    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
        
        // Linear actuator joint variables
        double actuator_joint_position_;
        double actuator_joint_velocity_;
        double actuator_joint_effort_;
        double actuator_joint_position_command_;

        // Ball screw joint variables
        double bscrew_joint_position_;
        double bscrew_joint_velocity_;
        double bscrew_joint_effort_;
        double bscrew_joint_position_command_;

        // Auger joint variables
        double auger_joint_position_;
        double auger_joint_velocity_;
        double auger_joint_effort_;
        double auger_joint_velocity_command_;
        
        // Wheel joint variables
        std::string wheel_joint_name_[2]={"wheel_joint_0","wheel_joint_1"};
        double wheel_joint_position_[2];
        double wheel_joint_velocity_[2];
        double wheel_joint_effort_[2];
        double wheel_joint_velocity_command_[2];
        
        ctre::phoenix::motorcontrol::can::TalonFX rightDriveFalcon;
        ctre::phoenix::motorcontrol::can::TalonFX leftDriveFalcon;
        ctre::phoenix::motorcontrol::can::TalonSRX linearActuatorTalon;
        ctre::phoenix::motorcontrol::can::TalonFX ballScrewFalcon;
        ctre::phoenix::motorcontrol::can::TalonFX augerFalcon;

        ros::NodeHandle nh_;
        ros::Timer my_control_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

        ros::Publisher bscrew_pos_pub;
        ros::Publisher actuator_pos_pub;
        ros::Publisher hit_limit_switch;
        void limitcallback(const std_msgs::Bool::ConstPtr& imsg);
};