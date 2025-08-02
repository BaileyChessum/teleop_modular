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
from typing import Dict, Iterator, Tuple
from teleop_python_utils.Button import Button
from teleop_python_utils.EventCollection import EventCollection

class ButtonCollection:
    def __init__(self, events: EventCollection):
        """ Constructor. You shouldn't use this directly, but rather through the Inputs class
        :param events: The EventCollection from the Inputs class
        """
        self.__items: Dict[str, Button] = {}
        self.__events: EventCollection = events

    def __getitem__(self, name: str) -> Button:
        """ Gets the button with a given name.
        Creates a button with value 0 if it does not yet exist.
        :param name: The name of the button to get
        :returns: The event with that name
        """
        if name in self.__items:
            return self.__items[name]

        # Make a new event if it doesn't already exist
        item = Button(name, self.__events)
        self.__items[name] = item
        return item

    def __iter__(self) -> Iterator[Tuple[str, Button]]:
        """ Allows you to iterate over every button, returning a tuple containing the name of the button, and the button
        :return: An iterator that returns a tuple containing a name and event for every event in the collection.
        """
        return ((name, self.__items[name]) for name in self.__items)
