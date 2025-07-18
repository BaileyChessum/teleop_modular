#ifndef TELEOP_MODULAR_CONTROL_MODE__VISIBILITY_CONTROL_H_
#define TELEOP_MODULAR_CONTROL_MODE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TELEOP_MODULAR_CONTROL_MODE_EXPORT __attribute__ ((dllexport))
    #define TELEOP_MODULAR_CONTROL_MODE_IMPORT __attribute__ ((dllimport))
  #else
    #define TELEOP_MODULAR_CONTROL_MODE_EXPORT __declspec(dllexport)
    #define TELEOP_MODULAR_CONTROL_MODE_IMPORT __declspec(dllimport)
  #endif
  #ifdef TELEOP_MODULAR_CONTROL_MODE_BUILDING_LIBRARY
    #define TELEOP_MODULAR_CONTROL_MODE_PUBLIC TELEOP_MODULAR_CONTROL_MODE_EXPORT
  #else
    #define TELEOP_MODULAR_CONTROL_MODE_PUBLIC TELEOP_MODULAR_CONTROL_MODE_IMPORT
  #endif
  #define TELEOP_MODULAR_CONTROL_MODE_PUBLIC_TYPE TELEOP_MODULAR_CONTROL_MODE_PUBLIC
  #define TELEOP_MODULAR_CONTROL_MODE_LOCAL
#else
  #define TELEOP_MODULAR_CONTROL_MODE_EXPORT __attribute__ ((visibility("default")))
  #define TELEOP_MODULAR_CONTROL_MODE_IMPORT
  #if __GNUC__ >= 4
    #define TELEOP_MODULAR_CONTROL_MODE_PUBLIC __attribute__ ((visibility("default")))
    #define TELEOP_MODULAR_CONTROL_MODE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TELEOP_MODULAR_CONTROL_MODE_PUBLIC
    #define TELEOP_MODULAR_CONTROL_MODE_LOCAL
  #endif
  #define TELEOP_MODULAR_CONTROL_MODE_PUBLIC_TYPE
#endif

#endif  // TELEOP_MODULAR_CONTROL_MODE__VISIBILITY_CONTROL_H_
