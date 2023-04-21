#pragma once

#include <memory>
#include <ros/ros.h>

class StateMachine;

class State {
public:
  State(ros::NodeHandle nh) : nh_(nh){};
  typedef std::shared_ptr<StateMachine> StateMachinePtr;

  virtual void run(StateMachinePtr machine) = 0;
  virtual void enter(StateMachinePtr machine) = 0;
  virtual void exit(StateMachinePtr machine) = 0;

protected:
  ros::NodeHandle nh_;

private:
};