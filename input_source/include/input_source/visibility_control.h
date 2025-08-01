// Copyright 2025 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#ifndef INPUT_SOURCE__VISIBILITY_CONTROL_H_
#define INPUT_SOURCE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define INPUT_SOURCE_EXPORT __attribute__ ((dllexport))
    #define INPUT_SOURCE_IMPORT __attribute__ ((dllimport))
  #else
    #define INPUT_SOURCE_EXPORT __declspec(dllexport)
    #define INPUT_SOURCE_IMPORT __declspec(dllimport)
  #endif
  #ifdef INPUT_SOURCE_BUILDING_LIBRARY
    #define INPUT_SOURCE_PUBLIC INPUT_SOURCE_EXPORT
  #else
    #define INPUT_SOURCE_PUBLIC INPUT_SOURCE_IMPORT
  #endif
  #define INPUT_SOURCE_PUBLIC_TYPE INPUT_SOURCE_PUBLIC
  #define INPUT_SOURCE_LOCAL
#else
  #define INPUT_SOURCE_EXPORT __attribute__ ((visibility("default")))
  #define INPUT_SOURCE_IMPORT
  #if __GNUC__ >= 4
    #define INPUT_SOURCE_PUBLIC __attribute__ ((visibility("default")))
    #define INPUT_SOURCE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define INPUT_SOURCE_PUBLIC
    #define INPUT_SOURCE_LOCAL
  #endif
  #define INPUT_SOURCE_PUBLIC_TYPE
#endif

#endif  // INPUT_SOURCE__VISIBILITY_CONTROL_H_
