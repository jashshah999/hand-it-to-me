#include <state_machine/state_machine.hpp>

StateMachine::StateMachine() {

}

StateMachine::~StateMachine() {
    current_state_->exit(shared_from_this());
}

void StateMachine::setState(StatePtr new_state) {
    current_state_->exit(shared_from_this());
    current_state_ = new_state;
    current_state_->enter(shared_from_this());
}
