#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.qos import QoSProfile
from typing import Union, Optional, Dict, List

from teleop_msgs.msg import InputNames, InputValues

# We want to support both Nodes and LifecycleNodes
NodeType: Union[Node, LifecycleNode]

class Inputs:
    """
    Helper class for getting input from teleop_modular.
    """
    def __init__(self, node: NodeType, topic: str, value_topic: Optional[str], qos: Union[QoSProfile, int, None]) -> None:
        """
        Constructor for the Inputs class.

        :param node: The Node or LifecycleNode to add input subscriptions to.
        :param topic: The name of the InputNames topic to get input from.
        :param value_topic: The name of the InputValues topic. Defaults to topic + "/values".
        :param qos: The Quality of Service (QoS) setting, which could be either an
                    instance of QoSProfile, an integer, or None. Defaults to an integer
                    value of 10 if not specified.
        """
        self.node: NodeType = node
        self.topic: str = topic

        if value_topic is None:
            value_topic = topic + "/values"

        self.value_topic: str = value_topic

        if qos is None:
            qos = 10

        self.qos: Union[QoSProfile, int] = qos

        self.axes: Dict[str, float] = {}
        self.buttons: Dict[str, int] = {}

        self.names_initialized = False
        self.axis_names: List[str] = []
        self.button_names: List[str] = []

        self.name_subscription = node.create_subscription(InputNames, self.topic, self.input_names_callback, qos)
        self.values_subscription = node.create_subscription(InputValues, self.value_topic, self.input_values_callback, qos)

    def input_names_callback(self, msg: InputNames):
        self.axis_names = msg.axis_names
        self.button_names = msg.button_names

        self.names_initialized = True

    def input_values_callback(self, msg: InputValues):

        for i in range(min(len(msg.axes), len(self.axis_names))):
            name = self.axis_names[i]
            self.axes[name] = msg.axes[i]

        for i in range(min(len(msg.buttons), len(self.button_names))):
            name = self.axis_names[i]
            self.buttons[name] = msg.axes[i]

        self.names_initialized = True
