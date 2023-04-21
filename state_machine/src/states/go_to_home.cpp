#include <state_machine/states/go_to_home.hpp>
#include <std_srvs/Empty.h>

GoToHome::GoToHome(ros::NodeHandle nh) : State(nh)
{
    ROS_INFO("GoToHome::GoToHome");
    client_ = nh_.serviceClient<std_srvs::Empty>("go_to_home");
    sub_ = nh_.subscribe("can_pose", 1, &GoToHome::canCallback, this);
};

void GoToHome::enter(StateMachinePtr machine)
{
    ROS_INFO("GoToHome::enter");
}

void GoToHome::run(StateMachinePtr machine)
{
    ROS_INFO("GoToHome::run");
    std_srvs::EmptyRequest request;
    bool success_ = client_.call(request);
    if (success_)
    {
        if (can_detected_)
        {
            machine->setState(ReachDroppedObject::getInstance(nh_));
        }
    }
}

void GoToHome::canCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{   
    if (first_message_)
    {
        last_can_detected_ = ros::Time::now();
        last_can_pose_ = *msg;
        first_message_ = false;
    }
    double diff = sqrt(pow(last_can_pose_.pose.position.x - msg->pose.position.x, 2) 
                     + pow(last_can_pose_.pose.position.y - msg->pose.position.y, 2) 
                     + pow(last_can_pose_.pose.position.z - msg->pose.position.z, 2));
    if (diff < 0.1)
    {
        last_can_detected_ = ros::Time::now();
        last_can_pose_ = *msg;
    }
    if (ros::Time::now() - last_can_detected_ > can_lost_timeout_)
    {
        can_detected_ = true;
    }
}

void GoToHome::exit(StateMachinePtr machine)
{
    ROS_INFO("GoToHome::exit");
    first_message_ = true;
}
