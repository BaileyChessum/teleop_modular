// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
//
// Created by nova on 7/2/25.
//

#include "teleop_core/inputs/InputCollection.hpp"

namespace teleop
{

// TODO(BaileyChessum): Make these redundant
template<>
void InputCollection<Button>::setup_new_item(const std::shared_ptr<Button> & item)
{
}

template<>
void InputCollection<Axis>::setup_new_item(const std::shared_ptr<Axis> & item)
{
}

}  // namespace teleop
