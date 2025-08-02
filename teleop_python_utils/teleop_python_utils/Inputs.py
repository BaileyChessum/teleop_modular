#!/usr/bin/env python3
#
# Copyright 2025 Bailey Chessum
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Created by Bailey Chessum on 2/8/25.
#
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.qos import QoSProfile
from typing import Union, Optional, Dict, List

from teleop_msgs.msg import InputNames, InputValues, InvokedEvents, CombinedInputValues, CombinedInputs
from teleop_msgs.msg import Inputs as InputsMessage

from teleop_python_utils.teleop_python_utils.EventCollection import Event, EventCollection

# We want to support both Nodes and LifecycleNodes
NodeType: Union[Node, LifecycleNode]
QoSType: Union[QoSProfile, int]


class Inputs:
    """
    Helper class for getting input values and event invocations from teleop_modular.
    """
    DEFAULT_NAMES_QOS = 10
    DEFAULT_VALUES_QOS = 10
    DEFAULT_EVENTS_QOS = 10
    DEFAULT_SPARSE_QOS = 10

    def __init__(self, node: NodeType) -> None:
        """
        Constructor for the Inputs class.

        You should call other fluent constructors after this! (e.g: .with_topics("name_topic", "name_topic/values"))

        :param node: The Node or LifecycleNode to add input subscriptions to.
        """
        self.node: NodeType = node

        self.axes: Dict[str, float] = {}
        self.buttons: Dict[str, int] = {}
        self.events: EventCollection = EventCollection()

        # Invoked whenever a message is received
        self.on_update = Event()

        # Just holds references to any subscription that get made, to keep them alive
        self.__subscriptions = []

        # For the split InputNames / InputValues configuration
        self.__names_subscription_created = False
        self.__names_initialized = False
        self.__axis_names: List[str] = []
        self.__button_names: List[str] = []

    def with_topics(self,
                    name_topic: str,
                    combined_values_topic: Optional[str],
                    values_qos: Union[QoSProfile, int, None] = DEFAULT_VALUES_QOS,
                    names_qos: Union[QoSProfile, int, None] = DEFAULT_NAMES_QOS
                    ) -> "Inputs":
        """ Alias for with_combined_topics, to help suggest this as the default fluent constructor to use.

        :return: The current object, so you can continue to chain calls to fluent constructor methods
        """
        self.with_combined_topics(name_topic, combined_values_topic, values_qos, names_qos)
        return self

    # Fluent Constructors
    def with_combined_topics(self,
                             name_topic: str,
                             combined_values_topic: Optional[str],
                             values_qos: QoSType = DEFAULT_VALUES_QOS,
                             names_qos: QoSType = DEFAULT_NAMES_QOS,
                             ) -> "Inputs":
        """ Fluent constructor method to accept an InputNames topic and a CombinedInputValues topic to receive values
        and events from.

        :return: The current object, so you can continue to chain calls to fluent constructor methods
        """
        if self.__assert_names_subscription_being_created_for_first_time():
            return self
        if combined_values_topic is None:
            combined_values_topic = name_topic + "/values"
        self.__subscriptions += [
            self.node.create_subscription(InputNames, name_topic, self.__input_names_callback, names_qos),
            self.node.create_subscription(CombinedInputValues, combined_values_topic, self.__combined_values_callback, values_qos)
        ]
        return self

    def with_frequent_topics(self,
                             name_topic: str,
                             values_topic: Optional[str],
                             values_qos: QoSType = DEFAULT_VALUES_QOS,
                             names_qos: QoSType = DEFAULT_NAMES_QOS
                             ) -> "Inputs":
        """ Fluent constructor method to accept an InputNames topic and an InputValues topic to receive values from.
        This message type doesn't include event invocations.

        :return: The current object, so you can continue to chain calls to fluent constructor methods
        """
        if self.__assert_names_subscription_being_created_for_first_time():
            return self
        if values_topic is None:
            values_topic = name_topic + "/values"
        self.__subscriptions += [
            self.node.create_subscription(InputNames, name_topic, self.__input_names_callback, names_qos),
            self.node.create_subscription(InputValues, values_topic, self.__input_values_callback, values_qos)
        ]
        return self

    def with_events_topic(self,
                          topic: str,
                          qos: QoSType = DEFAULT_EVENTS_QOS
                          ) -> "Inputs":
        """ Fluent constructor method to accept an InvokedEvents topic, to invoke events in self.events

        :return: The current object, so you can continue to chain calls to fluent constructor methods
        """
        self.__subscriptions += [
            self.node.create_subscription(InvokedEvents, topic, self.__invoked_events_callback, qos),
        ]
        return self

    def with_sparse_topic(self,
                          topic: str,
                          qos: QoSType = DEFAULT_SPARSE_QOS
                          ) -> "Inputs":
        """ Fluent constructor method to accept an Inputs topic, which defined the names of every input, along with
        input values. This allows inputs to be provided sparsely.

        :return: The current object, so you can continue to chain calls to fluent constructor methods
        """
        self.__subscriptions += [
            self.node.create_subscription(InputsMessage, topic, self.__sparse_inputs_callback, qos)
        ]
        return self

    def __assert_names_subscription_being_created_for_first_time(self) -> bool:
        """ Tries setting self.__names_subscription_create to True, but if it has already been set, logs a cranky
        message and returns True
        :return: True if the subscription has already been created. False if everything is fine.
        """
        if self.__names_subscription_created:
            self.node.get_logger().error("Two fluent constructors for a teleop_modular Input that both set the received"
                                         " input names were called! Please only use one source of input names.")
            return True

        self.__names_subscription_created = True
        return False

    #


    # Subscription Callbacks

    def __process_input_values(self, values: InputValues):
        if not self.__names_initialized:
            self.node.get_logger().debug("Didn't set input values because the name topic hasn't sent a message yet.")
            return

        for i in range(min(len(values.axes), len(self.__axis_names))):
            name = self.__axis_names[i]
            self.axes[name] = values.axes[i]

        for i in range(min(len(values.buttons), len(self.__button_names))):
            name = self.__axis_names[i]
            self.buttons[name] = values.axes[i]

    def __process_invoked_events(self, events: InvokedEvents):
        for event_name in events.names:
            self.events[event_name].invoke_silently()
        for event_name in events.names:
            self.events[event_name].invoke()

    def __input_names_callback(self, msg: InputNames):
        self.__axis_names = msg.axis_names
        self.__button_names = msg.button_names

        self.__names_initialized = True

    def __input_values_callback(self, msg: InputValues):
        self.__process_input_values(msg)
        self.on_update()

    def __invoked_events_callback(self, msg: InvokedEvents):
        self.__process_invoked_events(msg)
        self.on_update()

    def __combined_values_callback(self, msg: CombinedInputValues):
        self.__process_input_values(msg.values)
        self.__process_invoked_events(msg.events)
        self.on_update()

    def __sparse_inputs_callback(self, msg: InputsMessage):

        for i in range(min(len(msg.axis_names), len(msg.axes))):
            name = msg.axis_names[i]
            self.axes[name] = msg.axes[i]

        for i in range(min(len(msg.button_names), len(msg.buttons))):
            name = self.__axis_names[i]
            self.buttons[name] = msg.axes[i]

        self.on_update()
