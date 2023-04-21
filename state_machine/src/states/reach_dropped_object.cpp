#include "state_machine/states/reach_dropped_object.hpp"
#include <Eigen/Geometry>

ReachDroppedObject::ReachDroppedObject(ros::NodeHandle nh) : State(nh) {
  object_pose_sub_ = nh_.subscribe(
      "/can_pose", 1, &ReachDroppedObject::objectPoseCallback, this);
  robot_pose_sub_ = nh_.subscribe("/robot_pose", 1,
                                  &ReachDroppedObject::robotPoseCallback, this);
  goal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1);
}

void ReachDroppedObject::enter(StateMachinePtr machine) {
  ROS_INFO("Entering ReachDroppedObject state");
  active_ = true;
}

void ReachDroppedObject::exit(StateMachinePtr machine) {
  ROS_INFO("Exiting ReachDroppedObject state");
  active_ = false;
}

void ReachDroppedObject::run(StateMachinePtr machine) {
  ROS_INFO("Running ReachDroppedObject state");

  Eigen::Quaterniond object_quat(
      object_pose_.pose.orientation.w, object_pose_.pose.orientation.x,
      object_pose_.pose.orientation.y, object_pose_.pose.orientation.z);

  Eigen::Quaterniond robot_quat(
      robot_pose_.pose.orientation.w, robot_pose_.pose.orientation.x,
      robot_pose_.pose.orientation.y, robot_pose_.pose.orientation.z);

  Eigen::Vector3d object_pose(object_pose_.pose.position.x,
                              object_pose_.pose.position.y,
                              object_pose_.pose.position.z);

  Eigen::Vector3d robot_pose(robot_pose_.pose.position.x,
                             robot_pose_.pose.position.y,
                             robot_pose_.pose.position.z);

  Eigen::Vector3d object_to_robot_pose = robot_pose - object_pose;
  object_to_robot_pose[2] = object_to_robot_pose[2] - 0.05;

  if (object_to_robot_pose.norm() < 0.01 ||
      object_quat.angularDistance(robot_quat) < 0.1) {
    machine->setState(Pickup::getInstance(nh_));
  }
}
void ReachDroppedObject::objectPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  if (active_) {
    object_pose_ = *msg;
    goal_pose_pub_.publish(*msg);
  }
}

void ReachDroppedObject::robotPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  if (active_) {
    robot_pose_ = *msg;
  }
}