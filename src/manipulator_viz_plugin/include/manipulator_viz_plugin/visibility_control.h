#ifndef MANIPULATOR_VIZ_PLUGIN__VISIBILITY_CONTROL_H_
#define MANIPULATOR_VIZ_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MANIPULATOR_VIZ_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define MANIPULATOR_VIZ_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define MANIPULATOR_VIZ_PLUGIN_EXPORT __declspec(dllexport)
    #define MANIPULATOR_VIZ_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef MANIPULATOR_VIZ_PLUGIN_BUILDING_LIBRARY
    #define MANIPULATOR_VIZ_PLUGIN_PUBLIC MANIPULATOR_VIZ_PLUGIN_EXPORT
  #else
    #define MANIPULATOR_VIZ_PLUGIN_PUBLIC MANIPULATOR_VIZ_PLUGIN_IMPORT
  #endif
  #define MANIPULATOR_VIZ_PLUGIN_PUBLIC_TYPE MANIPULATOR_VIZ_PLUGIN_PUBLIC
  #define MANIPULATOR_VIZ_PLUGIN_LOCAL
#else
  #define MANIPULATOR_VIZ_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define MANIPULATOR_VIZ_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define MANIPULATOR_VIZ_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define MANIPULATOR_VIZ_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MANIPULATOR_VIZ_PLUGIN_PUBLIC
    #define MANIPULATOR_VIZ_PLUGIN_LOCAL
  #endif
  #define MANIPULATOR_VIZ_PLUGIN_PUBLIC_TYPE
#endif

#endif  // MANIPULATOR_VIZ_PLUGIN__VISIBILITY_CONTROL_H_
