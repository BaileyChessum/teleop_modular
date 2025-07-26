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
// Created by Bailey Chessum on 26/7/25.
//

#ifndef TELEOP_MODULAR_MAKE_ARRAY_HPP
#define TELEOP_MODULAR_MAKE_ARRAY_HPP

namespace teleop::utils {

/// Added to be a C++ 17 compatible replacement for C++ 20's std::make_array
template <typename... T>
constexpr auto make_array(T&&... t) -> std::array<std::common_type_t<T...>, sizeof...(T)> {
  return {std::forward<T>(t)...};
}

}  // namespace teleop::utils

#endif  // TELEOP_MODULAR_MAKE_ARRAY_HPP