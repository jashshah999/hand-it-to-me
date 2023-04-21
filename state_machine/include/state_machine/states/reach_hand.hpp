#pragma once

#include "state_machine/states/states_list.hpp"
#include <state_machine/state.hpp>
#include <std_msgs/Float32.h>

class ReachHand : public State {
public:
  ReachHand(ros::NodeHandle nh) : State(nh) {}
  void enter(StateMachinePtr state);
  void exit(StateMachinePtr state);
  void run(StateMachinePtr state);
  static StateMachine::StatePtr getInstance(ros::NodeHandle nh) {
    static std::shared_ptr<ReachHand> singleton =
        std::make_shared<ReachHand>(nh);
    return singleton;
  }

  void robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void handPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void gripperForceCallback(const std_msgs::Float32::ConstPtr &msg);

private:
  bool active_ = false;
  ros::Publisher goal_pose_pub_;
  ros::Subscriber hand_pose_sub_;
  ros::Subscriber robot_pose_sub_;
  ros::Subscriber gripper_force_sub_;

  geometry_msgs::PoseStamped hand_pose_;
  geometry_msgs::PoseStamped robot_pose_;
  std_msgs::Float32 gripper_force_;
};