// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//

#include <gtest/gtest.h>
#include "teleop_core/inputs/InputManager.hpp"
#include "teleop_core/inputs/input_pipeline_builder.hpp"

using teleop::InputManager;
using teleop::InputPipelineBuilder;
using teleop::InputPipelineElementDelegate;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * Pipeline element that produces one named axis by declaring an aggregate over a raw float.
 * Tracks how many times on_inputs_available has been called.
 */
class ProducerElement : public InputPipelineBuilder::Element
{
public:
  std::string name;
  float value = 0.0f;
  int on_inputs_available_count = 0;

  explicit ProducerElement(std::string name) : name(std::move(name)) {}

  void link_inputs(
    const InputManager::Props & previous, InputManager::Props & next,
    const DeclaredNames &) override
  {
    next = previous;
    next.axis_builder.declare_aggregate(name, &value);
  }

  void on_inputs_available(InputManager::Hardened &) override
  {
    ++on_inputs_available_count;
  }
};

/**
 * Pipeline element that:
 *   - declares it wants to consume a named axis (declare_input_names)
 *   - captures the Axis InputPtr in on_inputs_available
 *   - contributes nothing to link_inputs (pure consumer)
 *
 * Models a ButtonEvent / control mode that binds inputs during on_inputs_available.
 */
class ConsumerElement : public InputPipelineBuilder::Element
{
public:
  std::string name;
  control_mode::Axis bound;
  int on_inputs_available_count = 0;

  explicit ConsumerElement(std::string name) : name(std::move(name)) {}

  void declare_input_names(DeclaredNames & names) override
  {
    names.axis_names.insert(name);
  }

  void link_inputs(
    const InputManager::Props & previous, InputManager::Props & next,
    const DeclaredNames &) override
  {
    next = previous;
  }

  void on_inputs_available(InputManager::Hardened & inputs) override
  {
    bound = inputs.axes[name];
    ++on_inputs_available_count;
  }
};

/**
 * A collection element analogous to EventCollection: it holds child Elements, forwards
 * declare_input_names / on_inputs_available to them, and calls request_full_pipeline_relink()
 * when a child is added after the pipeline has been linked.
 *
 * Also implements InputPipelineElementDelegate so children can call relink through it.
 */
class DynamicCollection : public InputPipelineBuilder::Element, public InputPipelineElementDelegate
{
public:
  bool pipeline_linked = false;
  std::vector<InputPipelineBuilder::Element *> children;

  void add_child(InputPipelineBuilder::Element * child)
  {
    children.push_back(child);
    child->set_pipeline_delegate(*this);
    if (pipeline_linked) {
      request_full_pipeline_relink();
    }
  }

  // InputPipelineElementDelegate — children call relink() on us; we propagate upward
  void relink() override { relink_pipeline(); }
  void request_full_relink() override { request_full_pipeline_relink(); }

  void declare_input_names(DeclaredNames & names) override
  {
    for (auto * child : children) {
      child->declare_input_names(names);
    }
  }

  void link_inputs(
    const InputManager::Props & previous, InputManager::Props & next,
    const DeclaredNames & names) override
  {
    next = previous;
    for (auto * child : children) {
      child->link_inputs(previous, next, names);
    }
  }

  void on_inputs_available(InputManager::Hardened & inputs) override
  {
    for (auto * child : children) {
      child->on_inputs_available(inputs);
    }
    pipeline_linked = true;
  }
};

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

class DynamicPipelineTest : public ::testing::Test
{
protected:
  InputManager inputs;
  InputPipelineBuilder pipeline{inputs};

  void SetUp() override
  {
    inputs = InputManager();
    pipeline.clear();
  }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// flush_pending_relink() when nothing was requested is a no-op.
TEST_F(DynamicPipelineTest, FlushWithNoPendingRelink_IsNoop)
{
  ProducerElement producer("axis");
  pipeline.push_back(producer);
  pipeline.link_inputs();

  const int count_before = producer.on_inputs_available_count;
  pipeline.flush_pending_relink();
  pipeline.flush_pending_relink();

  EXPECT_EQ(producer.on_inputs_available_count, count_before)
    << "flush_pending_relink() without a request should not trigger on_inputs_available";
}

// request_full_relink() is deferred: nothing happens until flush_pending_relink() is called.
TEST_F(DynamicPipelineTest, RequestFullRelink_IsDeferredUntilFlush)
{
  ProducerElement producer("axis");
  pipeline.push_back(producer);
  pipeline.link_inputs();

  producer.value = 42.0f;
  inputs.update(rclcpp::Time());

  const int count_before = producer.on_inputs_available_count;
  pipeline.request_full_relink();

  // Not yet — no relink should have happened
  EXPECT_EQ(producer.on_inputs_available_count, count_before)
    << "request_full_relink() must not trigger a relink immediately";

  pipeline.flush_pending_relink();

  EXPECT_GT(producer.on_inputs_available_count, count_before)
    << "flush_pending_relink() must trigger on_inputs_available after a request";
}

// After flush, the rebuilt pipeline still reads the correct value.
TEST_F(DynamicPipelineTest, AfterFlush_InputValuesAreCorrect)
{
  ProducerElement producer("axis");
  pipeline.push_back(producer);
  pipeline.link_inputs();

  producer.value = 7.0f;
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["axis"].value(), 7.0f, 1e-6f);

  pipeline.request_full_relink();
  pipeline.flush_pending_relink();

  producer.value = 13.0f;
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(inputs.get_axes()["axis"].value(), 13.0f, 1e-6f)
    << "Input reads should still work correctly after a full relink";
}

// Multiple requests between flushes result in exactly one relink.
TEST_F(DynamicPipelineTest, MultipleRequests_OnlyOneRelink)
{
  ProducerElement producer("axis");
  pipeline.push_back(producer);
  pipeline.link_inputs();

  const int count_before = producer.on_inputs_available_count;
  pipeline.request_full_relink();
  pipeline.request_full_relink();
  pipeline.request_full_relink();

  pipeline.flush_pending_relink();

  EXPECT_EQ(producer.on_inputs_available_count, count_before + 1)
    << "Multiple requests between flushes should collapse into one relink";
}

// A ConsumerElement added before link_inputs has its inputs bound on initial link.
TEST_F(DynamicPipelineTest, ConsumerAddedBeforeLink_IsBound)
{
  ProducerElement producer("axis");
  DynamicCollection collection;
  ConsumerElement consumer("axis");

  collection.add_child(&consumer);

  pipeline.push_back(producer);
  pipeline.push_back(collection);
  pipeline.link_inputs();

  EXPECT_TRUE(bool(consumer.bound)) << "Consumer added before link should have a bound input";

  producer.value = 5.0f;
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(consumer.bound.value(), 5.0f, 1e-6f);
}

// A ConsumerElement added to a DynamicCollection AFTER the pipeline is linked must have
// its input properly declared and bound after flush_pending_relink().
TEST_F(DynamicPipelineTest, ConsumerAddedAfterLink_BoundAfterFlush)
{
  ProducerElement producer("axis");
  DynamicCollection collection;

  pipeline.push_back(producer);
  pipeline.push_back(collection);
  pipeline.link_inputs();

  // Now dynamically add a consumer — this is the CR2-5 scenario
  ConsumerElement late_consumer("axis");
  collection.add_child(&late_consumer);

  // Before flush: consumer has not received on_inputs_available yet
  EXPECT_EQ(late_consumer.on_inputs_available_count, 0)
    << "Consumer should not be bound before flush";
  EXPECT_FALSE(bool(late_consumer.bound))
    << "Consumer input should not be bound before flush";

  // Simulate the update thread calling flush at the top of the next cycle
  pipeline.flush_pending_relink();

  EXPECT_EQ(late_consumer.on_inputs_available_count, 1)
    << "Consumer should have received on_inputs_available after flush";
  EXPECT_TRUE(bool(late_consumer.bound))
    << "Consumer input must be bound after flush";

  producer.value = 3.0f;
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(late_consumer.bound.value(), 3.0f, 1e-6f)
    << "Dynamically added consumer must read correct input values after flush";
}

// Existing consumers survive a full relink triggered by adding a new child.
TEST_F(DynamicPipelineTest, ExistingConsumer_SurvivesFullRelink)
{
  ProducerElement producer("axis");
  DynamicCollection collection;
  ConsumerElement existing_consumer("axis");

  collection.add_child(&existing_consumer);

  pipeline.push_back(producer);
  pipeline.push_back(collection);
  pipeline.link_inputs();

  // Verify the existing consumer works before the dynamic add
  producer.value = 1.0f;
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(existing_consumer.bound.value(), 1.0f, 1e-6f);

  // Add a new consumer — triggers deferred full relink
  ConsumerElement late_consumer("axis");
  collection.add_child(&late_consumer);
  pipeline.flush_pending_relink();

  // Existing consumer must still be bound and reading correctly
  producer.value = 9.0f;
  inputs.update(rclcpp::Time());
  EXPECT_NEAR(existing_consumer.bound.value(), 9.0f, 1e-6f)
    << "Existing consumer must survive a full relink triggered by a dynamic add";
  EXPECT_NEAR(late_consumer.bound.value(), 9.0f, 1e-6f)
    << "New consumer must also read the correct value";
}

// Two consumers for different input names both get bound after adding a second producer.
TEST_F(DynamicPipelineTest, TwoConsumers_DifferentNames_BothBoundAfterFlush)
{
  ProducerElement producer_a("axis_a");
  ProducerElement producer_b("axis_b");
  DynamicCollection collection;
  ConsumerElement consumer_a("axis_a");
  ConsumerElement consumer_b("axis_b");

  collection.add_child(&consumer_a);

  pipeline.push_back(producer_a);
  pipeline.push_back(producer_b);
  pipeline.push_back(collection);
  pipeline.link_inputs();

  EXPECT_TRUE(bool(consumer_a.bound));

  // Dynamically add consumer for the second axis
  collection.add_child(&consumer_b);
  pipeline.flush_pending_relink();

  producer_a.value = 2.0f;
  producer_b.value = 8.0f;
  inputs.update(rclcpp::Time());

  EXPECT_NEAR(consumer_a.bound.value(), 2.0f, 1e-6f);
  EXPECT_NEAR(consumer_b.bound.value(), 8.0f, 1e-6f);
}

// flush_pending_relink() after clear() (no elements) does not crash.
TEST_F(DynamicPipelineTest, FlushAfterClear_NoCrash)
{
  pipeline.request_full_relink();
  pipeline.clear();
  // After clear, the flag is reset — flush should be a no-op
  pipeline.flush_pending_relink();
}
