#ifndef TELEOP_MODULAR_TWIST__VISIBILITY_CONTROL_H_
#define TELEOP_MODULAR_TWIST__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TELEOP_MODULAR_TWIST_EXPORT __attribute__ ((dllexport))
    #define TELEOP_MODULAR_TWIST_IMPORT __attribute__ ((dllimport))
  #else
    #define TELEOP_MODULAR_TWIST_EXPORT __declspec(dllexport)
    #define TELEOP_MODULAR_TWIST_IMPORT __declspec(dllimport)
  #endif
  #ifdef TELEOP_MODULAR_TWIST_BUILDING_LIBRARY
    #define TELEOP_MODULAR_TWIST_PUBLIC TELEOP_MODULAR_TWIST_EXPORT
  #else
    #define TELEOP_MODULAR_TWIST_PUBLIC TELEOP_MODULAR_TWIST_IMPORT
  #endif
  #define TELEOP_MODULAR_TWIST_PUBLIC_TYPE TELEOP_MODULAR_TWIST_PUBLIC
  #define TELEOP_MODULAR_TWIST_LOCAL
#else
  #define TELEOP_MODULAR_TWIST_EXPORT __attribute__ ((visibility("default")))
  #define TELEOP_MODULAR_TWIST_IMPORT
  #if __GNUC__ >= 4
    #define TELEOP_MODULAR_TWIST_PUBLIC __attribute__ ((visibility("default")))
    #define TELEOP_MODULAR_TWIST_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TELEOP_MODULAR_TWIST_PUBLIC
    #define TELEOP_MODULAR_TWIST_LOCAL
  #endif
  #define TELEOP_MODULAR_TWIST_PUBLIC_TYPE
#endif

#endif  // TELEOP_MODULAR_TWIST__VISIBILITY_CONTROL_H_
