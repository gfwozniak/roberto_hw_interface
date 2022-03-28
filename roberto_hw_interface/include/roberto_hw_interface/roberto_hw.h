#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

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
        void initJoints();
        void initDrive();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        
    protected:
        hardware_interface::JointStateInterface joint_state_interface_; // All joint states 
        hardware_interface::VelocityJointInterface velocity_joint_interface_;
//        hardware_interface::PositionJointInterface position_joint_interface_;
//        hardware_interface::EffortJointInterface effort_joint_interface_;
        
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
//        joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
//        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
        
        std::string joint_name_[2]={"left_wheel_joint","right_wheel_joint"};  
        double joint_position_[2];
        double joint_velocity_[2];
        double joint_effort_[2]; // right motor 3, left motor 4
        double joint_velocity_command_[2]; // right motor 0, left motor 1
        
        ctre::phoenix::motorcontrol::can::TalonFX rightDriveTalon;
        ctre::phoenix::motorcontrol::can::TalonFX leftDriveTalon;
        std::string interface = "can0";

        ros::NodeHandle nh_;
        ros::Timer my_control_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};