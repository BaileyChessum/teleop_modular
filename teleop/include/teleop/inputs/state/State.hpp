//
// Created by nova on 7/4/25.
//

#ifndef STATE_HPP
#define STATE_HPP

#include <memory>
#include <string>
#include "teleop/inputs/InputDeclaration.hpp"

namespace teleop {

/// Same as a InputDeclaration, but it actually holds the value being referenced too.
template<typename T>
struct State : InputDeclaration<T> {
  using SharedPtr = std::shared_ptr<State>;

  /// Value offered up as the declaration.
  T value;
  // TODO: Timeout

  State(const std::string& name, T initial_value) : InputDeclaration<T>(name, value), value(initial_value) {}
};

} // teleop

#endif //STATE_HPP
