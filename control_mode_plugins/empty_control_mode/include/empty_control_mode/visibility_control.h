// Copyright 2026 Bailey Chessum
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
#ifndef EMPTY_CONTROL_MODE__VISIBILITY_CONTROL_H_
#define EMPTY_CONTROL_MODE__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EMPTY_CONTROL_MODE_EXPORT __attribute__ ((dllexport))
    #define EMPTY_CONTROL_MODE_IMPORT __attribute__ ((dllimport))
  #else
    #define EMPTY_CONTROL_MODE_EXPORT __declspec(dllexport)
    #define EMPTY_CONTROL_MODE_IMPORT __declspec(dllimport)
  #endif
  #ifdef EMPTY_CONTROL_MODE_BUILDING_LIBRARY
    #define EMPTY_CONTROL_MODE_PUBLIC EMPTY_CONTROL_MODE_EXPORT
  #else
    #define EMPTY_CONTROL_MODE_PUBLIC EMPTY_CONTROL_MODE_IMPORT
  #endif
  #define EMPTY_CONTROL_MODE_PUBLIC_TYPE EMPTY_CONTROL_MODE_PUBLIC
  #define EMPTY_CONTROL_MODE_LOCAL
#else
  #define EMPTY_CONTROL_MODE_EXPORT __attribute__ ((visibility("default")))
  #define EMPTY_CONTROL_MODE_IMPORT
  #if __GNUC__ >= 4
    #define EMPTY_CONTROL_MODE_PUBLIC __attribute__ ((visibility("default")))
    #define EMPTY_CONTROL_MODE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EMPTY_CONTROL_MODE_PUBLIC
    #define EMPTY_CONTROL_MODE_LOCAL
  #endif
  #define EMPTY_CONTROL_MODE_PUBLIC_TYPE
#endif

#endif  // EMPTY_CONTROL_MODE__VISIBILITY_CONTROL_H_
