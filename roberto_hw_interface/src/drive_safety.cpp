#include <roberto_hw_interface/drive_safety.h>

DriveSafety::Republisher::Republisher(ros::Publisher * publisher)
: outputTwist(publisher)
{
	// Read parameter server
	ros::param::get("~bscrew_max", bscrew_max);
	ros::param::get("~bscrew_min", bscrew_min);
	ros::param::get("~actuator_max", actuator_max);
	ros::param::get("~actuator_min", actuator_min);
	zeroOutput.linear.x = 0;
	zeroOutput.linear.y = 0;
	zeroOutput.linear.z = 0;
	zeroOutput.angular.x = 0;
	zeroOutput.angular.y = 0;
	zeroOutput.angular.z = 0;
}

// Conditions to activate drivetrain
bool DriveSafety::Republisher::isDriveable()
{
	return (bscrew_position < bscrew_max &&
		bscrew_position > bscrew_min &&
		actuator_position < actuator_max &&
		actuator_position > actuator_min);
}

// Main callback method for Twist input
void DriveSafety::Republisher::callback(const geometry_msgs::Twist::ConstPtr& imsg)
{
	// Publishes a zero velocity message if the drivetrain is not driveable
	if (isDriveable())
		outputTwist->publish(imsg);
	else
		outputTwist->publish(zeroOutput);
}

// Callback to read actuator position
void DriveSafety::Republisher::callbackActuatorPosition(const std_msgs::Float64::ConstPtr& imsg)
{
	actuator_position = imsg->data;
}

// Callback to read ballscrew position
void DriveSafety::Republisher::callbackBScrewPosition(const std_msgs::Float64::ConstPtr& imsg)
{
	bscrew_position = imsg->data;
}

int main(int argc, char **argv)
{
	// Node initialize
	ros::init(argc, argv, "drive_safety");
	ros::NodeHandle n;

	// Initialize publishing on topic "output" with message type std_msgs/Float64
	ros::Publisher output = n.advertise<geometry_msgs::Twist>("output", 1000);

	DriveSafety::Republisher outputObj(&output);

	// Initialize subscribing on topic "input" to call with callback method in Repub instance
	ros::Subscriber inputTwist = n.subscribe("input", 1000, &DriveSafety::Republisher::callback, &outputObj);
	ros::Subscriber inputBScrewPosition = n.subscribe("ballscrew_pos", 1000, &DriveSafety::Republisher::callbackBScrewPosition, &outputObj);
	ros::Subscriber inputActuatorPosition = n.subscribe("actuator_pos", 1000, &DriveSafety::Republisher::callbackActuatorPosition, &outputObj);

	// Spin (pump callbacks for subscribe function)
	ros::spin();

	return 0;
}