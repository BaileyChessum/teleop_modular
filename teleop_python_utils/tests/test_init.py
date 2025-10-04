#!/usr/bin/env python3
from teleop_python_utils import Inputs
from rclpy.node import Node
import rclpy

def test_dummy():
    rclpy.init()
    Inputs(Node("test"))
    assert True