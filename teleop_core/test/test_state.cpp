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
// Created by nova on 5/7/25.
//

#include <gtest/gtest.h>
#include "teleop_core/inputs/Button.hpp"
#include "teleop_core/inputs/Axis.hpp"
#include "teleop_core/inputs/InputManager.hpp"
#include "teleop_core/inputs/state/State.hpp"
#include "teleop_core/inputs/state/StateCollection.hpp"
#include "teleop_core/inputs/state/StateManager.hpp"

using teleop::Axis;
using teleop::Button;
using teleop::InputManager;
using teleop::state::State;
using teleop::state::StateCollection;
using teleop::state::StateManager;

class StateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Setup code that will be called before each test
  }

  void TearDown() override
  {
    // Cleanup code that will be called after each test
  }
};

TEST_F(StateTest, StateButton)
{
//  InputManager inputs;
//  StateCollection<uint8_t, Button> states{inputs.get_buttons()};
//
//  EXPECT_EQ(inputs.get_buttons()["test_button"].value(), 0);
//  EXPECT_EQ(inputs.get_buttons()["test_button"].value(), false);
//  EXPECT_FALSE(inputs.get_buttons()["test_button"].value());
//  inputs.update(rclcpp::Time(0));
//  EXPECT_FALSE(inputs.get_buttons()["test_button"].value());
//
//  // Set value via state
//  states.set("test_button", true);
//  inputs.update(rclcpp::Time(100));
//  EXPECT_TRUE(inputs.get_buttons()["test_button"].value());
//
//  states.set("test_button", false);
//  inputs.update(rclcpp::Time(200));
//  EXPECT_FALSE(inputs.get_buttons()["test_button"].value());
//
//  states.set("test_button", true);
//  inputs.update(rclcpp::Time(300));
//  EXPECT_TRUE(inputs.get_buttons()["test_button"].value());
//
//  states.clear("test_button");
//  inputs.update(rclcpp::Time(400));
//  EXPECT_FALSE(inputs.get_buttons()["test_button"].value());
}

TEST_F(StateTest, StateAxis)
{
//  InputManager inputs;
//  StateCollection<float, Axis> states{inputs.get_axes()};
//
//  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 0.0, 1e-10);
//  inputs.update(rclcpp::Time(0));
//  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 0.0, 1e-10);
//
//  // Set value via state
//  states.set("test_axis", 1.0);
//  inputs.update(rclcpp::Time(100));
//  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 1.0, 1e-10);
//
//  states.set("test_axis", 0.0);
//  inputs.update(rclcpp::Time(200));
//  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 0.0, 1e-10);
//
//  states.set("test_axis", 2.0);
//  inputs.update(rclcpp::Time(300));
//  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 2.0, 1e-10);
//
//  states.clear("test_axis");
//  inputs.update(rclcpp::Time(400));
//  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 0.0, 1e-10);
}
