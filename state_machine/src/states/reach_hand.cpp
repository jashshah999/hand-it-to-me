#include "state_machine/states/reach_hand.hpp"
#include <Eigen/Dense>

ReachHand::ReachHand(ros::NodeHandle nh) : State(nh) {
  hand_pose_sub_ =
      nh_.subscribe("/hand_pose", 1, &ReachHand::handPoseCallback, this);
  robot_pose_sub_ =
      nh_.subscribe("/robot_pose", 1, &ReachHand::robotPoseCallback, this);
  gripper_force_sub_ = nh_.subscribe("/force_on_gripper", 1,
                                     &ReachHand::gripperForceCallback, this);
  goal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1);
}

void ReachHand::enter(StateMachinePtr machine) {
  ROS_INFO("Entering ReachHand state");
  active_ = true;
}

void ReachHand::exit(StateMachinePtr machine) {
  ROS_INFO("Exiting ReachHand state");
  active_ = false;
}

void ReachHand::run(StateMachinePtr machine) {
  ROS_INFO("Running ReachHand state");

  Eigen::Quaterniond hand_quat(
      hand_pose_.pose.orientation.w, hand_pose_.pose.orientation.x,
      hand_pose_.pose.orientation.y, hand_pose_.pose.orientation.z);

  Eigen::Quaterniond robot_quat(
      robot_pose_.pose.orientation.w, robot_pose_.pose.orientation.x,
      robot_pose_.pose.orientation.y, robot_pose_.pose.orientation.z);

  Eigen::Vector3d hand_pose(hand_pose_.pose.position.x,
                            hand_pose_.pose.position.y,
                            hand_pose_.pose.position.z);

  Eigen::Vector3d robot_pose(robot_pose_.pose.position.x,
                             robot_pose_.pose.position.y,
                             robot_pose_.pose.position.z);

  Eigen::Vector3d hand_to_robot_pose = hand_pose - robot_pose;

  if (hand_to_robot_pose.norm() < 0.01 ||
      hand_quat.angularDistance(robot_quat) < 0.1) {
    machine->setState(HandOverObject::getInstance(nh_));
  }

  if (gripper_force_.data < 0.5) {
    machine->setState(GoToHome::getInstance(nh_));
  }
}

void ReachHand::robotPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  if (active_) {
    robot_pose_ = *msg;
  }
}

void ReachHand::handPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  if (active_) {
    hand_pose_ = *msg;
    goal_pose_pub_.publish(*msg);
  }
}

void ReachHand::gripperForceCallback(const std_msgs::Float32::ConstPtr &msg) {
  if (active_) {
    gripper_force_ = *msg;
  }
}