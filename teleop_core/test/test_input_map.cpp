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
// Created by nova on 27/8/25.
//

#include <gtest/gtest.h>
#include "teleop_core/inputs/Button.hpp"
#include "teleop_core/inputs/Axis.hpp"
#include "teleop_core/inputs/InputManager.hpp"
#include "teleop_core/events/EventManager.hpp"

using teleop::Axis;
using teleop::Button;
using teleop::InputManager;
using teleop::internal::EventManager;
using teleop::InputMap;
using teleop::InputMapBuilder;

class InputTest : public ::testing::Test
{
public:
  InputManager inputs;
  InputManager::Props props;

protected:
  void SetUp() override
  {
    // Setup code that will be called before each test
    inputs = InputManager();
    props = InputManager::Props();
  }

  void TearDown() override
  {
    // Cleanup code that will be called after each test
  }
};

TEST_F(InputTest, MapButtonSimple)
{
  EXPECT_FALSE(inputs.get_buttons()["test_button"].value());
  EXPECT_EQ(inputs.get_buttons()["test_button"].get(), Button().get());

  uint8_t value = false;
  props.button_builder.declare_aggregate("test_button", &value);
  inputs.init(props);
  EXPECT_NE(inputs.get_buttons()["test_button"].get(), Button().get()) << "Still nullptr after assignment";

  inputs.update(rclcpp::Time());
  EXPECT_FALSE(inputs.get_buttons()["test_button"].value());

  value = true;
  inputs.update(rclcpp::Time());
  EXPECT_TRUE(inputs.get_buttons()["test_button"].value());
}

TEST_F(InputTest, MapAxisSimple)
{
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 0.0, 1e-10);

  float value = 1.0f;
  props.axis_builder.declare_aggregate("test_axis", &value);
  inputs.init(props);

  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 1.0, 1e-10);

  value = 0.0;
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 0.0, 1e-10);

  value = 0.5;
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 0.5, 1e-10);
}

TEST_F(InputTest, AxisSpamInit)
{
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 0.0, 1e-10);
  std::cerr << "0\n";

  float value = 1.0;
  props.axis_builder.declare_aggregate("test_axis", &value);
  std::cerr << "1.1\n";
  inputs.init(props);
  std::cerr << "1.2\n";
  inputs.init(props);
  std::cerr << "1.3\n";
  inputs.init(props);
  std::cerr << "1.4\n";
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 1.0, 1e-10);

  float value2 = 10.0;
  props.axis_builder.declare_aggregate("test_axis", &value2);
  std::cerr << "2.1\n";
  inputs.init(props);
  std::cerr << "2.2\n";
  inputs.init(props);
  std::cerr << "2.3\n";
  inputs.init(props);
  std::cerr << "2.4\n";
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 11.0, 1e-10);

  float value3 = 10.0;
  props.axis_builder.declare_aggregate("test_axis", &value3);
  std::cerr << "3.1\n";
  inputs.init(props);
  std::cerr << "3.2\n";
  inputs.init(props);
  std::cerr << "3.3\n";
  inputs.init(props);
  std::cerr << "3.4\n";
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 11.0, 1e-10);
}

TEST_F(InputTest, MapAxisDependencyAccumulationDirect)
{
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 0.0, 1e-10);
  std::cerr << "0\n";

  float value = 1.0;
  props.axis_builder.declare_aggregate("test_axis", &value);
  float value2 = 10.0;
  props.axis_builder.declare_aggregate("test_axis", &value2);
  float value3 = 100.0;
  props.axis_builder.declare_aggregate("test_axis", &value3);
  std::cerr << "1\n";
  inputs.init(props);
  std::cerr << "2\n";

  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 111.0, 1e-10);
  std::cerr << "3\n";

  inputs.init(props);
  std::cerr << "4\n";
  inputs.init(props);
  std::cerr << "5\n";
  inputs.init(props);
  std::cerr << "6\n";
}

TEST_F(InputTest, MapAxisDependencyAccumulation)
{
  std::cerr << "0\n";
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 0.0, 1e-10);
  std::cerr << "0\n";

  float value = 1.0;
  props.axis_builder.declare_aggregate("test_axis", &value);
  inputs.init(props);
  std::cerr << "0\n";

  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 1.0, 1e-10);
  std::cerr << "1\n";

  float value2 = 10.0;
  props.axis_builder.declare_aggregate("test_axis", &value2);
  inputs.init(props);
  std::cerr << "2\n";

  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 11.0, 1e-10);
  std::cerr << "3\n";

  value = 3.0;
  value2 = 10.0;
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 13.0, 1e-10);
  std::cerr << "4\n";

  value = 3.0;
  value2 = 30.0;
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 33.0, 1e-10);
  std::cerr << "5\n";

  value = 1.0;
  value2 = 10.0;
  float value3 = 100.0;
  std::cerr << "5.1\n";
  props.axis_builder.declare_aggregate("test_axis", &value3);
  std::cerr << "5.2\n";
  inputs.init(props);
  std::cerr << "6\n";

  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 111.0, 1e-10);
  std::cerr << "7\n";

  value = 3.0;
  value2 = 30.0;
  value3 = 100.0;
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 133.0, 1e-10);
  std::cerr << "8\n";

  value = 1.0;
  value2 = 10.0;
  value3 = 300.0;
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 311.0, 1e-10);
  std::cerr << "9\n";

  value = 3.0;
  value2 = 30.0;
  value3 = 300.0;
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 333.0, 1e-10);
  std::cerr << "10\n";

  value = 1.0;
  value2 = 10.0;
  value3 = 100.0;
  float value4 = 1000.0;
  props.axis_builder.declare_aggregate("test_axis", &value4);
  inputs.init(props);
  std::cerr << "11\n";

  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["test_axis"].value(), 1111.0, 1e-10);
  std::cerr << "12\n";
}

TEST_F(InputTest, MapButtonDependencyAccumulation)
{
  EXPECT_FALSE(inputs.get_buttons()["test_button"].value());

  uint8_t value = false;
  props.button_builder.declare_aggregate("test_button", &value);
  inputs.init(props);

  inputs.update(rclcpp::Time());
  EXPECT_FALSE(inputs.get_buttons()["test_button"].value());

  value = true;
  inputs.update(rclcpp::Time());
  EXPECT_TRUE(inputs.get_buttons()["test_button"].value());

  value = false;
  uint8_t value2 = false;
  props.button_builder.declare_aggregate("test_button", &value2);
  inputs.init(props);

  inputs.update(rclcpp::Time());
  EXPECT_FALSE(inputs.get_buttons()["test_button"].value());

  value = true;
  value2 = false;
  inputs.update(rclcpp::Time());
  EXPECT_TRUE(inputs.get_buttons()["test_button"].value());

  value = false;
  value2 = true;
  inputs.update(rclcpp::Time());
  EXPECT_TRUE(inputs.get_buttons()["test_button"].value());

  value = true;
  value2 = true;
  inputs.update(rclcpp::Time());
  EXPECT_TRUE(inputs.get_buttons()["test_button"].value());

  value = false;
  value2 = false;
  inputs.update(rclcpp::Time());
  EXPECT_FALSE(inputs.get_buttons()["test_button"].value());
}

