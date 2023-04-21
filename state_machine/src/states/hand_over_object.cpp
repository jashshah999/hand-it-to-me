#include <state_machine/states/hand_over_object.hpp>

HandOverObject::HandOverObject(ros::NodeHandle nh) : State(nh)
{
    ROS_INFO("HandOverObject::HandOverObject");
    client_ = nh.serviceClient<std_srvs::Empty>("hand_over_object");
    sub_ = nh_.subscribe("force_on_gripper", 1, &HandOverObject::forceCallback, this);
};

void HandOverObject::enter(StateMachinePtr machine)
{
    ROS_INFO("HandOverObject::enter");
};

void HandOverObject::run(StateMachinePtr machine)
{
    ROS_INFO("HandOverObject::run");
    if (force_ > max_force_)
    {
        std_srvs::EmptyRequest request;
        success_ = client_.call(request);
        if (success_)
        {
            machine->setState(GoToHome::getInstance(nh_));
        }
    }
    else if (force_ < min_force_)
    {
        machine->setState(GoToHome::getInstance(nh_));
    }
};

void HandOverObject::forceCallback(const std_msgs::Float32::ConstPtr &msg)
{
    force_ = msg->data;
};

void HandOverObject::exit(StateMachinePtr machine)
{
    ROS_INFO("HandOverObject::exit");
}
