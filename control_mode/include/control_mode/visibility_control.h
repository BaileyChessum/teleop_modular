#ifndef CONTROL_MODE__VISIBILITY_CONTROL_H_
#define CONTROL_MODE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CONTROL_MODE_EXPORT __attribute__ ((dllexport))
    #define CONTROL_MODE_IMPORT __attribute__ ((dllimport))
  #else
    #define CONTROL_MODE_EXPORT __declspec(dllexport)
    #define CONTROL_MODE_IMPORT __declspec(dllimport)
  #endif
  #ifdef CONTROL_MODE_BUILDING_LIBRARY
    #define CONTROL_MODE_PUBLIC CONTROL_MODE_EXPORT
  #else
    #define CONTROL_MODE_PUBLIC CONTROL_MODE_IMPORT
  #endif
  #define CONTROL_MODE_PUBLIC_TYPE CONTROL_MODE_PUBLIC
  #define CONTROL_MODE_LOCAL
#else
  #define CONTROL_MODE_EXPORT __attribute__ ((visibility("default")))
  #define CONTROL_MODE_IMPORT
  #if __GNUC__ >= 4
    #define CONTROL_MODE_PUBLIC __attribute__ ((visibility("default")))
    #define CONTROL_MODE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONTROL_MODE_PUBLIC
    #define CONTROL_MODE_LOCAL
  #endif
  #define CONTROL_MODE_PUBLIC_TYPE
#endif

#endif  // CONTROL_MODE__VISIBILITY_CONTROL_H_
