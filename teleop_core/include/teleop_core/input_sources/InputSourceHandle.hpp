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
// Created by nova on 7/6/25.
//

#ifndef TELEOP_MODULAR_INPUTSOURCEHANDLE_HPP
#define TELEOP_MODULAR_INPUTSOURCEHANDLE_HPP

#include <memory>
#include <utility>
#include <vector>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include "input_source/input_source.hpp"
#include "teleop_core/utilities/better_multimap.hpp"
#include "teleop_core/inputs/InputManager.hpp"

namespace teleop::internal
{

/// For each InputSource we store, we associate some extra metadata.
class InputSourceHandle
{
  using ParameterInterface = rclcpp::node_interfaces::NodeParametersInterface;

public:
  explicit InputSourceHandle(
    const ParameterInterface::SharedPtr & parameters, InputManager & inputs,
    const std::shared_ptr<input_source::InputSource> & source);

  explicit InputSourceHandle(
    InputManager & inputs,
    const std::shared_ptr<input_source::InputSource> & source);

  void update(const rclcpp::Time & now);

  void add_definitions_to_inputs() const;

  void declare_and_link_inputs();

private:
  struct ButtonTransformParams
  {
    bool invert = false;
  };
  struct AxisTransformParams
  {
    bool invert = false;

    struct Range
    {
      std::array<float, 2> in = {-1.0f, 1.0f};
      std::optional<std::array<float, 2>> out;
      bool clamp = false;
    };

    std::optional<Range> range;
    std::optional<float> power;
  };

  struct ButtonFromAxisParams
  {
    std::string name;
    float threshold = 0.0;
  };
  struct AxisFromButtonsParams
  {
    std::optional<std::string> positive;
    std::optional<std::string> negative;
  };

  template<typename transformT, typename fromOtherParamsT>
  struct RemapRenameParams
  {
    std::string name;
    std::optional<std::string> from;
    std::optional<transformT> transform;
    std::optional<fromOtherParamsT> from_other;
  };

  using RemapButtonParams = RemapRenameParams<ButtonTransformParams, ButtonFromAxisParams>;
  using RemapAxisParams = RemapRenameParams<AxisTransformParams, AxisFromButtonsParams>;

  struct RemapParams
  {
    std::vector<RemapButtonParams> buttons;
    std::vector<RemapAxisParams> axes;
  };

  /// The thing that actually holds the result of remap transformations in memory
  template<typename T, typename fromOtherT, typename transformT>
  struct TransformedRemapValue
  {
    T value;
    std::optional<T*> from;
    std::optional<fromOtherT> from_other;
    std::optional<transformT> transform;

    TransformedRemapValue(
      T value, std::optional<T*> from, std::optional<fromOtherT> from_other,
      std::optional<transformT> transform)
    : value(value), from(from), from_other(from_other), transform(transform)
    {
    }

    inline void update(const rclcpp::Time & now);
  };

  struct TransformedRemapButtonFromAxis
  {
    float* axis;
    float threshold = 0.0f;
  };
  struct TransformedRemapAxisFromButtons
  {
    std::optional<uint8_t*> negative;
    std::optional<uint8_t*> positive;
  };

  using TransformedRemapButton = TransformedRemapValue<uint8_t, TransformedRemapButtonFromAxis,
      ButtonTransformParams>;
  using TransformedRemapAxis = TransformedRemapValue<float, TransformedRemapAxisFromButtons,
      AxisTransformParams>;

  /// This is an actual connection of an input to a defining value
  template<typename T>
  struct Definition
  {
    std::string name;
    T* ptr;

    Definition(std::string name, T* ptr) : name(std::move(name)), ptr(ptr) {}
  };

  void remap(input_source::InputDeclarationSpans declarations, RemapParams remap_params);
  RemapParams get_remap_params();

  std::optional<RemapButtonParams> get_remap_button_params(const std::string & name);
  std::optional<ButtonTransformParams> get_button_transform_params(const std::string & name);

  std::optional<RemapAxisParams> get_remap_axis_params(const std::string & name);
  std::optional<AxisTransformParams> get_axis_transform_params(const std::string & name);

  std::vector<Definition<uint8_t>> button_definitions_;
  std::vector<Definition<float>> axis_definitions_;

  std::vector<TransformedRemapButton> transformed_buttons_;
  std::vector<TransformedRemapAxis> transformed_axes_;

  std::shared_ptr<input_source::InputSource> source_;
  std::reference_wrapper<InputManager> inputs_;
  ParameterInterface::SharedPtr parameters_;
};

}  // namespace teleop::internal

#endif  // TELEOP_MODULAR_INPUTSOURCEHANDLE_HPP
