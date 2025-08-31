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
// Created by Bailey Chessum on 27/8/25.
//

#include <gtest/gtest.h>
#include "teleop_core/inputs/Button.hpp"
#include "teleop_core/inputs/Axis.hpp"
#include "teleop_core/inputs/InputManager.hpp"
#include "teleop_core/events/EventManager.hpp"
#include "teleop_core/inputs/input_pipeline_builder.hpp"

using teleop::Axis;
using teleop::Button;
using teleop::InputManager;
using teleop::internal::EventManager;
using teleop::InputMap;
using teleop::InputMapBuilder;
using teleop::InputPipelineBuilder;
using teleop::InputPipelineElementDelegate;

class InputPipelineTest : public ::testing::Test, public InputPipelineBuilder::Element
{
public:
  InputManager inputs;
  InputManager::Props props;
  InputPipelineBuilder pipeline = InputPipelineBuilder(inputs);

protected:
  void SetUp() override
  {
    // Setup code that will be called before each test
    inputs = InputManager();
    props = InputManager::Props();

    pipeline.clear();
    pipeline.push_back(*this);
  }

  void TearDown() override
  {
    // Cleanup code that will be called after each test
  }

  void link_inputs(const InputManager::Props &previous, InputManager::Props &next, const std::set<std::string> &declared_names) override {
    next = props;
  }

  virtual void on_inputs_available(InputManager::Hardened& inputs) override {

  }
};

TEST_F(InputPipelineTest, SimplePipeline)
{
  EXPECT_FALSE(inputs.get_buttons()["test_button"].value());
  EXPECT_EQ(inputs.get_buttons()["test_button"].get(), Button().get());

  uint8_t value = false;
  props.button_builder.declare_aggregate("test_button", &value);
  pipeline.link_inputs();

  EXPECT_NE(inputs.get_buttons()["test_button"].get(), Button().get()) << "Still nullptr after assignment";

  inputs.update(rclcpp::Time());
  EXPECT_FALSE(inputs.get_buttons()["test_button"].value());

  value = true;
  inputs.update(rclcpp::Time());
  EXPECT_TRUE(inputs.get_buttons()["test_button"].value());
}


