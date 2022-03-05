#include <roberto_hw_interface/topic_convert.h>

Repub::Repub(ros::Publisher * publisher)
: output(publisher)
{
    // Nothing, constructor
}

// Callback method
void Repub::callback(const geometry_msgs::Twist::ConstPtr& imsg)
{
    std_msgs::Float64 omsg;
    omsg.data = imsg->linear.x;
    output->publish(omsg);
}

int main(int argc, char **argv)
{
  // Node initialize
  ros::init(argc, argv, "topic_convert");
  ros::NodeHandle n;

  // Initialize publishing on topic "output" with message type std_msgs/Float64
  ros::Publisher output = n.advertise<std_msgs::Float64>("output", 1000);

  // So this was weird about roscpp, you're supposed to create
  // an object with a method to process the callback. This was
  // the simplest way I could think to have the subscribe callback 
  // function publish on a topic.
  Repub outputObj(&output);

  // Initialize subscribing on topic "input" to call with callback method in Repub instance
  ros::Subscriber input = n.subscribe("input", 1000, &Repub::callback, &outputObj);

  // Spin (pump callbacks for subscribe function)
  ros::spin();

  return 0;
}