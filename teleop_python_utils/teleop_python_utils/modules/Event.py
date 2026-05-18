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
from typing import List, Callable, ParamSpec, Generic

P = ParamSpec('P')  # Generic type for 'any parameters'
Callback = Callable[P, None]    # You can have events and callbacks with 'any parameters',
# but teleop will only ever have events without ANY parameters.
# This is only if you wanted to use your own custom events for whatever reason. It's a handy type!


class Event(Generic[P]):
    """ An object you can add callbacks to, and invoke to call all the assigned callbacks.
    In teleop, you can add callbacks for when a teleop event is invoked, and check if the event was invoked for this
    update by checking .is_invoked.

    You can check if a button was pressed this update by getting inputs.events["button_name/down"].is_invoked,
    or if a button was released by getting inputs.events["button_name/up"] example.
    """
    def __init__(self):
        """ Constructor

        :param call_callbacks_on_reset: When true, the event will call callbacks when reset() is called
        """
        self.callbacks: List[Callback[P]] = []
        self.is_invoked: bool = False

    def __bool__(self):
        """ Helper function to get if an event is invoked. """
        return self.is_invoked

    def add_callback(self, callback: Callback):
        """ Adds a function to be called whenever the event is invoked. """
        self.callbacks.append(callback)

    def remove_callback(self, callback: Callback):
        """ Removes a previously added callback function. """
        self.callbacks.remove(callback)

    def invoke(self, *args: P.args, **kwargs: P.kwargs) -> None:
        """ Calls all the callback methods.
        You can use it if you want to make your own Events.

        Call with any arguments you want to call the callback functions with.
        The args and kwargs just mean 'forward any arguments to callback'

        :param args: Positional arguments accepted by the callbacks
        :param kwargs: Keyword arguments accepted by the callbacks
        :return: None
        """
        self.is_invoked = True
        for callback in self.callbacks:
            callback(*args, **kwargs)

    def __call__(self, *args: P.args, **kwargs: P.kwargs) -> None:
        """ Shorthand for .invoke() """
        self.invoke(*args, **kwargs)

    def invoke_silently(self) -> None:
        """ Sets is_invoked without triggering the callbacks.
        Only use this if you know what you are doing! Don't call this for teleop events.
        """
        self.is_invoked = True

    def reset_invocation(self) -> None:
        """ Sets is_invoked back to false
        Only use this if you know what you are doing! Don't call this for teleop events.
        """
        self.is_invoked = False

