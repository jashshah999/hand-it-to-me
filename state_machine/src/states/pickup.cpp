#include "state_machine/states/pickup.hpp"

Pickup::Pickup(ros::NodeHandle nh) : State(nh) {
  client_ = nh_.serviceClient<std_srvs::Empty>("/pickup");
}

void Pickup::enter(StateMachinePtr machine) {
  ROS_INFO("Entering Pickup state");
}

void Pickup::exit(StateMachinePtr machine) { ROS_INFO("Exiting Pickup state"); }

void Pickup::run(StateMachinePtr machine) {
  ROS_INFO("Running Pickup state");
  std_srvs::Empty srv;

  if (client_.call(srv)) {
    machine->setState(ReachHand::getInstance(nh_));
  } else {
    machine->setState(GoToHome::getInstance(nh_));
  }
}