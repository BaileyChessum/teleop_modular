// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
//
// Created by Bailey Chessum on 13/8/25.
//

#ifndef TELEOP_MODULAR_INPUT_TRANSFORMATION_HPP
#define TELEOP_MODULAR_INPUT_TRANSFORMATION_HPP

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/node.hpp>
#include "input_transformation/visibility_control.h"
#include "input_transformation/indirect_span.hpp"
#include <cstdint>

namespace input_transformation
{

/**
 * To replace returning bools to indicate whether an operation was successful or failed due to some error. Used to
 * be more explicit.
 */
enum class INPUT_TRANSFORMATION_PUBLIC_TYPE return_type : bool
{
  OK = false,
  ERROR = true,
};

struct INPUT_TRANSFORMATION_PUBLIC_TYPE InputValues {
  indirect_span<const uint8_t> buttons;
  indirect_span<const float> axes;
};

struct INPUT_TRANSFORMATION_PUBLIC_TYPE OutputValues {
  indirect_span<uint8_t> buttons;
  indirect_span<float> axes;
};

struct INPUT_TRANSFORMATION_PUBLIC_TYPE Names {
  std::vector<std::string> buttons;
  std::vector<std::string> axes;
};

/**
 * \class Base class for a control mode used in teleoperation.
 */
class INPUT_TRANSFORMATION_PUBLIC InputTransformation
{
public:
  InputTransformation() = default;

  /**
   * This function effectively replaces the initializer, and should only be called once immediately after spawning the
   * input transformation, and before anything else.
   */
  return_type init(const std::string & name);

  virtual return_type on_configure() = 0;
  virtual std::vector<std::string> on_export_inputs() = 0;
  virtual std::vector<std::string> on_export_outputs() = 0;

  virtual void on_update() = 0;

  // Accessors
  /**
   * \brief Gets the name of the control mode, which the control mode is indexed by.
   */
  [[nodiscard]] const std::string & get_name() const
  {
    return name_;
  }


protected:

private:
  /// The ROS2 node created by teleop_modular, which we get params from (for base and child classes)
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_{};
};

}  // namespace input_transformation

#endif  // TELEOP_MODULAR_INPUT_TRANSFORMATION_HPP
