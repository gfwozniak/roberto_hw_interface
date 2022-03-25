#include <roberto_hw_interface/diff_drive.h>

Repub::Repub(ros::Publisher * publisherRight, ros::Publisher * publisherLeft)
: outputLeft(publisherLeft),
outputRight(publisherRight)
{
  // Read parameter server
  ros::param::get("~totalMod", totalMod);
  ros::param::get("~leftMod", leftMod);
  ros::param::get("~rightMod", rightMod);
  ros::param::get("~turnMod", turnMod);
}

// Callback method
void Repub::callback(const geometry_msgs::Twist::ConstPtr& imsg)
{
  double x = imsg->linear.x;
  double z = imsg->angular.z;
  double rv;
  double lv;

  // Input multiplier
  x *= totalMod;
  z *= totalMod;

  // Differential drive equation
  lv = x - turnMod * z;
  rv = x + turnMod * z;

  // Modifiers for each wheel
  rv *= rightMod;
  lv *= leftMod;

  // Publish commands
  std_msgs::Float64 omsgLeft;
  std_msgs::Float64 omsgRight;
  omsgRight.data = rv;
  omsgLeft.data = lv;
  outputRight->publish(omsgRight);
  outputLeft->publish(omsgLeft);
}

int main(int argc, char **argv)
{
  // Node initialize
  ros::init(argc, argv, "topic_convert");
  ros::NodeHandle n;

  // Initialize publishing on topic "output" with message type std_msgs/Float64
  ros::Publisher outputLeft = n.advertise<std_msgs::Float64>("outputLeft", 1000);
  ros::Publisher outputRight = n.advertise<std_msgs::Float64>("outputRight", 1000);

  // So this was weird about roscpp, you're supposed to create
  // an object with a method to process the callback. This was
  // the simplest way I could think to have the subscribe callback 
  // function publish on a topic.
  Repub outputObj(&outputRight, &outputLeft);

  // Initialize subscribing on topic "input" to call with callback method in Repub instance
  ros::Subscriber input = n.subscribe("input", 1000, &Repub::callback, &outputObj);

  // Spin (pump callbacks for subscribe function)
  ros::spin();

  return 0;
}