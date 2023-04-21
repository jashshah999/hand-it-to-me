#pragma once

#include "state_machine/states/states_list.hpp"
#include <state_machine/state.hpp>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

class HandOverObject : public State {
public:
  HandOverObject(ros::NodeHandle nh);
  void enter(StateMachinePtr machine);
  void exit(StateMachinePtr machine);
  void run(StateMachinePtr machine);
  void forceCallback(const std_msgs::Float32::ConstPtr& msg);
  static StateMachine::StatePtr getInstance(ros::NodeHandle nh) {
    static std::shared_ptr<HandOverObject> singleton = std::make_shared<HandOverObject>(nh);
    return singleton;
  }

private:
  ros::Subscriber sub_;
  ros::ServiceClient client_;
  float force_;
  bool success_ = false;
  float min_force_ = 0.0;
  float max_force_ = 5.0;
};