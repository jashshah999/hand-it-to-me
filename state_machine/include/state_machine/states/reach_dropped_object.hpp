#pragma once

#include "state_machine/states/states_list.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <state_machine/state.hpp>

class ReachDroppedObject : public State {
public:
  ReachDroppedObject(ros::NodeHandle nh);
  void enter(StateMachinePtr machine);
  void exit(StateMachinePtr machine);
  void run(StateMachinePtr machine);
  static StateMachine::StatePtr getInstance(ros::NodeHandle nh) {
    static std::shared_ptr<ReachDroppedObject> singleton =
        std::make_shared<ReachDroppedObject>(nh);
    return singleton;
  }

  void objectPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

private:
  ros::Subscriber object_pose_sub_;
  ros::Subscriber robot_pose_sub_;
  ros::Publisher goal_pose_pub_;
  bool active_;

  geometry_msgs::PoseStamped object_pose_;
  geometry_msgs::PoseStamped robot_pose_;
};