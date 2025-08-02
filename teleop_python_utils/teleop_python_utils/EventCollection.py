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
from teleop_python_utils.teleop_python_utils.Event import Event


class EventCollection:
    """ A class used by teleop to access Events for a given name. If the event requested does not yet exist, it will be
    created upon request.
    """

    def __init__(self):
        """ Constructor """
        self.__events: Dict[str, Event] = {}

    def __getitem__(self, event_name: str) -> Event:
        """ Gets an event with a given name, or creates one and returns it if it doesn't already exist.
        :param event_name: The name of the event to get
        :returns: The event with that name
        """
        if event_name in self.__events:
            return self.__events[event_name]

        # Make a new event if it doesn't already exist
        event = Event()
        self.__events[event_name] = event
        return event

    def __setitem__(self, event_name: str, event: Event):
        """ Adds an event to the collection. You shouldn't need to use this. Just get an event with self["event_name"],
        then add callbacks to the return value of that.
        :param event_name: The name of the event to set
        :param event: The event to add
        :returns: The event with that name
        """
        if event_name not in self.__events:
            self.__events[event_name] = event
            return

        # Combine the two events if an event already exists
        existing_event = self.__events[event_name]
        for callback in existing_event.callbacks:
            event.add_callback(callback)
        self.__events[event_name] = event

    def __iter__(self) -> Iterator[Tuple[str, Event]]:
        """ Allows you to iterate over every event, returning a tuple containing the name of the event, and the event
        :return: An iterator that returns a tuple containing a name and event for every event in the collection.
        """
        return ((name, self.__events[name]) for name in self.__events)
