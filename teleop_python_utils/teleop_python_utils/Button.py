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
from teleop_python_utils.EventCollection import EventCollection


class Button:
    """ Helper class to attach on_pressed and on_released values onto Button values. You can treat this like a bool. """

    def __init__(self, name: str, events: EventCollection):
        """ Constructor. You shouldn't call this directly. Get it through the Input class instead.
        :param name: The name to get pressed and released events from.
        :param events: The EventCollection to get pressed and released events from.
        """
        self.value: int = 0

        # Helpers to just point to the correct name to use for button pressing and releasing
        self.on_down = events[name + "/down"]
        self.on_up = events[name + "/up"]

    def add_callback(self, callback):
        """ Shorthand for on_down.add_callback. Calls the given callback when the button BEGINS being pressed. """
        self.on_down.add_callback(callback)

    def __bool__(self) -> bool:
        """ Conversion to bool values, allowing it to be used as the condition in an if statement. """
        return self.value != 0

    def __int__(self) -> int:
        """ Shorthand access to self.value when converting with int() """
        return self.value

    def down(self) -> bool:
        """ Returns true when the button has just become true since the last update. """
        return self.on_down.is_invoked

    def up(self) -> bool:
        """ Returns true when the button has just become false since the last update. """
        return self.on_up.is_invoked
