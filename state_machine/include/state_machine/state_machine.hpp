#pragma once
#include "state_machine/state.hpp"

class StateMachine : public std::enable_shared_from_this<StateMachine> {
public:
  typedef std::shared_ptr<State> StatePtr;

  StateMachine();
  ~StateMachine();

  StatePtr getCurrentState() const { return current_state_; }
  void setState(StatePtr state);

private:
  StatePtr current_state_;

};