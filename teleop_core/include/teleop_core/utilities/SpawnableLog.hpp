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
// Created by nova on 7/5/25.
//

#ifndef TELEOP_MODULAR_SPAWNABLELOG_HPP
#define TELEOP_MODULAR_SPAWNABLELOG_HPP

#include <string>
#include <optional>
#include <utility>
#include <sstream>
#include "teleop_core/colors.hpp"

namespace teleop::utils
{

/**
 * Holds information necessary to log the status of spawning some spawnable object
 */
struct SpawnableLog
{
  std::string name;
  std::optional<std::string> type;
  /// Unset when succeeded. Set when there was an issue.
  std::optional<std::string> error = std::nullopt;

  explicit SpawnableLog(
    std::string name, const std::optional<std::string> & type = std::nullopt,
    const std::optional<std::string> & error = std::nullopt)
  : name(std::move(name)), type(type), error(error)
  {
  }

  [[nodiscard]] std::string to_string() const
  {
    std::stringstream output;

    if (error.has_value()) {
      output << C_FAIL_QUIET;
    } else {
      // Success color here. Left blank to use the default color.
    }

    // registered_sources_log << C_FAIL_QUIET << "\n\t- " << pretty_name << C_FAIL_QUIET << "\t(failed - can't find
    // plugin " << input_source_type << ") " << C_RESET;
    output << "\t- " << name;

    if (type.has_value() && !error.has_value()) {
      output << "\t" C_QUIET << ": " << type.value();
    }

    if (error.has_value()) {
      output << C_FAIL_QUIET "(failed - " << error.value() << ")";
    }

    output << C_RESET;
    return output.str();
  }
};

}  // namespace teleop::utils

#endif  // TELEOP_MODULAR_SPAWNABLELOG_HPP
