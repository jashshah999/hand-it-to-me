#pragma once

#include "state_machine/states/states_list.hpp"
#include <state_machine/state.hpp>

#include <std_srvs/Empty.h>

class Pickup : public State {
public:
  Pickup(ros::NodeHandle nh);
  void enter(StateMachinePtr machine);
  void exit(StateMachinePtr machine);
  void run(StateMachinePtr machine);
  static StateMachine::StatePtr getInstance(ros::NodeHandle nh) {
    static std::shared_ptr<Pickup> singleton = std::make_shared<Pickup>(nh);
    return singleton;
  }

private:
  ros::ServiceClient client_;
};