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
from teleop_python_utils.EventCollection import EventCollection
from teleop_python_utils.Axis import Axis

class AxisCollection:
    def __init__(self):
        """ Constructor. You shouldn't use this directly, but rather through the Inputs class
        """
        self.__items: Dict[str, Axis] = {}

    def __getitem__(self, name: str) -> float:
        """ Gets the axis with a given name.
        Creates an axis with value 0 if it does not yet exist.
        :param name: The name of the axis to get
        :returns: The event with that name
        """
        if name in self.__items:
            return self.__items[name].value

        # Make a new event if it doesn't already exist
        item = Axis()
        self.__items[name] = item
        return item.value

    def __iter__(self) -> Iterator[Tuple[str, Axis]]:
        """ Allows you to iterate over every axis, returning a tuple containing the name of the axis, and the axis
        :return: An iterator that returns a tuple containing a name and event for every event in the collection.
        """
        return ((name, self.__items[name]) for name in self.__items)

    def get(self, name: str) -> Axis:
        """ Gets an object that holds the value for an axis, to avoid the cost of string lookup every update. """
        if name in self.__items:
            return self.__items[name]

        # Make a new event if it doesn't already exist
        item = Axis()
        self.__items[name] = item
        return item
