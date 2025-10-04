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
#include <input_source/input_source.hpp>
#include <input_source/input_declaration_list.hpp>
#include <control_mode/control_mode.hpp>

namespace input_transformation
{

/**
 * \brief To replace returning bools to indicate whether an operation was successful or failed due to some error. Used
 * to be more explicit.
 */
using INPUT_TRANSFORMATION_PUBLIC_TYPE return_type = control_mode::return_type;

/**
 * \brief Provides references to all collections of input objects that might be needed in an input transformation.
 *
 * Including this extra struct affords abstraction for the providers of buttons, axes, and the potential expansion of
 * available inputs (events?)
 */
using INPUT_TRANSFORMATION_PUBLIC_TYPE Inputs = control_mode::Inputs;

/**
 * \brief Contains references to memory to put output values into. Memory is laid out the same as the declared button
 * and axis name orders.
 */
using INPUT_TRANSFORMATION_PUBLIC_TYPE InputValueSpans = input_source::InputValueSpans;

/**
 * \brief Base class for something that modifies inputs, or takes in some inputs, and outputs some other input values
 * based on the inputs it accepts.
 */
class INPUT_TRANSFORMATION_PUBLIC InputTransformation
{
public:
  InputTransformation() = default;

  /**
   * \brief This function effectively replaces the initializer, and should only be called once immediately after
   * spawning the input transformation, and before anything else.
   *
   * Since there is no dynamic parameter changing support for input transformations, on_configure() is called at the end
   * of init().
   */
  return_type init(const std::string & name);

  /**
   * \brief Called after initialization, allows the input transformation to get any configuration from the custom
   * transformations .yaml file entry or otherwise.
   */
  virtual return_type on_configure() = 0;

  // TODO: Implement a custom parameter provider for manually parsed YAML files, to avoid going through ROS2 params.
  /**
   * \brief Called after on_configure, allows the implementer to capture any inputs needed by the transformation.
   *
   * \param inputs References to the collection of buttons and axes to be used by the input transformation.
   */
  virtual void on_configure_inputs(Inputs inputs) = 0;

  /**
   * \brief Called after on_configure(), allows the input transformation to declare the names of all the button values
   * output by the input transformation.
   *
   * \returns The names of all buttons to provide outputs for. When the input values are later exposed to be set in
   * on_update, they will be in the same order as the names exported here.
   */
  virtual std::vector<std::string> export_buttons() = 0;

  /**
   * \brief Called after on_configure(), allows the input source to declare the names of all the axis values output by
   * the input transformation.
   *
   * \returns The names of all axes to provide outputs for. When the input values are later exposed to be set in
   * on_update, they will be in the same order as the names exported here.
   */
  virtual std::vector<std::string> export_axes() = 0;

  /**
   * Called when new input should be processed, after requesting an update through request_update().
   * @param[in] now The time of the update, for synchronisation.
   * @param[out] values Modified by the InputSource to set the values for each button and axis.
   */
  virtual return_type on_update(const rclcpp::Time & now, InputValueSpans values) = 0;

  // Accessors
  /**
   * \brief Gets the name of the input transformation, used for debugging purposes. Likely auto-generated from the input
   * transformation plugin type name.
   */
  [[nodiscard]] inline const std::string & get_name() const noexcept
  {
    return name_;
  }

  /**
   * \brief Accessor for the main teleop node
   */
  [[nodiscard]] inline const std::shared_ptr<rclcpp::Node>& get_node() const noexcept
  {
    return node_;
  }

protected:

private:
  /// The ROS2 node created by teleop_modular, which we get params from (for base and child classes)
  std::shared_ptr<rclcpp::Node> node_{};
  std::string name_{};
};

}  // namespace input_transformation

#endif  // TELEOP_MODULAR_INPUT_TRANSFORMATION_HPP
