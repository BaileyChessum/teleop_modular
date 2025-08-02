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
class Axis:
    """ Helper class to attach on_pressed and on_released values onto Axis values. """

    def __init__(self):
        """ Constructor. You shouldn't call this directly. Get it through the Input class instead.
        """
        self.value: float = 0

    def __float__(self) -> float:
        """ Shorthand access to self.value when converting with float(). """
        return self.value
