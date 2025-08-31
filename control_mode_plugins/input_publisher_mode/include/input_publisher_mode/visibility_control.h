#ifndef INPUT_PUBLISHER_MODE__VISIBILITY_CONTROL_H_
#define INPUT_PUBLISHER_MODE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define INPUT_PUBLISHER_MODE_EXPORT __attribute__ ((dllexport))
    #define INPUT_PUBLISHER_MODE_IMPORT __attribute__ ((dllimport))
  #else
    #define INPUT_PUBLISHER_MODE_EXPORT __declspec(dllexport)
    #define INPUT_PUBLISHER_MODE_IMPORT __declspec(dllimport)
  #endif
  #ifdef INPUT_PUBLISHER_MODE_BUILDING_LIBRARY
    #define INPUT_PUBLISHER_MODE_PUBLIC INPUT_PUBLISHER_MODE_EXPORT
  #else
    #define INPUT_PUBLISHER_MODE_PUBLIC INPUT_PUBLISHER_MODE_IMPORT
  #endif
  #define INPUT_PUBLISHER_MODE_PUBLIC_TYPE INPUT_PUBLISHER_MODE_PUBLIC
  #define INPUT_PUBLISHER_MODE_LOCAL
#else
  #define INPUT_PUBLISHER_MODE_EXPORT __attribute__ ((visibility("default")))
  #define INPUT_PUBLISHER_MODE_IMPORT
  #if __GNUC__ >= 4
    #define INPUT_PUBLISHER_MODE_PUBLIC __attribute__ ((visibility("default")))
    #define INPUT_PUBLISHER_MODE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define INPUT_PUBLISHER_MODE_PUBLIC
    #define INPUT_PUBLISHER_MODE_LOCAL
  #endif
  #define INPUT_PUBLISHER_MODE_PUBLIC_TYPE
#endif

#endif  // INPUT_PUBLISHER_MODE__VISIBILITY_CONTROL_H_
