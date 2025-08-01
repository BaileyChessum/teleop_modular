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
// Created by nova on 6/13/25.
//

#ifndef TELEOP_MODULAR_UTILS_HPP
#define TELEOP_MODULAR_UTILS_HPP

#include <string>
#include <string_view>

inline std::string snake_to_title(const std::string_view in)
{
  std::string out;
  out.reserve(in.size());  // 1‑for‑1 replacement, no re‑allocs

  bool new_word = true;  // first char should be capitalised
  for (const char ch : in) {
    if (ch == '_') { // underscore -> space, next char starts a word
      out.push_back(' ');
      new_word = true;
    } else {
      if (new_word && std::isalpha(static_cast<unsigned char>(ch))) {
        out.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));
      } else {
        out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
      }
      new_word = false;
    }
  }
  return out;
}


#endif  // TELEOP_MODULAR_UTILS_HPP
