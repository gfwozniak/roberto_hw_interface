#include <roberto_hw_interface/telemetry_republisher.h>
#include <string>

TelemetryRepublisher::Republisher::Republisher(std::vector<ros::Publisher> * publishers)
: output(publishers)
{
    // Nothing, constructor
}

// Callback method
void TelemetryRepublisher::Republisher::callback(const sensor_msgs::JointState::ConstPtr& imsg)
{
    std::vector<std_msgs::Float64> msg(6);

    msg[0].data = imsg->position.at(0);
    msg[1].data = imsg->position.at(2);
    msg[2].data = imsg->velocity.at(1);
    msg[3].data = imsg->position.at(1);
    msg[4].data = imsg->effort.at(1);
    msg[5].data = imsg->effort.at(0);

    for (int i = 0; i < 6; ++i)
    {
      output->at(i).publish(msg[i]);
    }
}

int main(int argc, char **argv)
{
  // Node initialize
  ros::init(argc, argv, "topic_convert");
  ros::NodeHandle n;

  std::vector<std::string> names = { "actuator_position",
    "bscrew_position",
    "auger_velocity",
    "auger_current",
    "auger_temperature",
    "battery_voltage" };
  int namecount = names.size();

  // Initialize publishing on topic "output" with message type std_msgs/Float64

  std::vector<ros::Publisher> publishers;
  for (int i = 0; i < namecount; ++i)
  {
    publishers.push_back(n.advertise<std_msgs::Float64>(names[i], 1000));
  }

  // So this was weird about roscpp, you're supposed to create
  // an object with a method to process the callback. This was
  // the simplest way I could think to have the subscribe callback 
  // function publish on a topic.
  TelemetryRepublisher::Republisher outputObj(&publishers);

  // Initialize subscribing on topic "input" to call with callback method in Repub instance
  ros::Subscriber input = n.subscribe("joint_states", 1000, &TelemetryRepublisher::Republisher::callback, &outputObj);

  // Spin (pump callbacks for subscribe function)
  ros::spin();

  return 0;
}