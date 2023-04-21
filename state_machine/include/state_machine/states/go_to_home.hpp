#pragma once

#include "state_machine/states/states_list.hpp"
#include <state_machine/state.hpp>
#include <state_machine/state_machine.hpp>
#include <geometry_msgs/PoseStamped.h>

class GoToHome : public State {
public:

  GoToHome(ros::NodeHandle nh);
  void enter(StateMachinePtr machine);
  void exit(StateMachinePtr machine);
  void run(StateMachinePtr machine);
  void canCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  static StateMachine::StatePtr getInstance(ros::NodeHandle nh) {
    static std::shared_ptr<GoToHome> singleton = std::make_shared<GoToHome>(nh);
    return singleton;
  }

private:
  ros::ServiceClient client_;
  ros::Subscriber sub_;
  bool can_detected_ = false;
  bool first_message_ = true;
  bool success_ = false;
  ros::Time last_can_detected_;
  ros::Duration can_lost_timeout_ = ros::Duration(1.0);
  geometry_msgs::PoseStamped last_can_pose_;
};