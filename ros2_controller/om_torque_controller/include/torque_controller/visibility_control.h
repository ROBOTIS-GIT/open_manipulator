#ifndef TORQUE_CONTROLLER__VISIBILITY_CONTROL_H_
#define TORQUE_CONTROLLER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TORQUE_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define TORQUE_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define TORQUE_CONTROLLER_EXPORT __declspec(dllexport)
    #define TORQUE_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef TORQUE_CONTROLLER_BUILDING_DLL
    #define TORQUE_CONTROLLER_PUBLIC TORQUE_CONTROLLER_EXPORT
  #else
    #define TORQUE_CONTROLLER_PUBLIC TORQUE_CONTROLLER_IMPORT
  #endif
  #define TORQUE_CONTROLLER_PUBLIC_TYPE TORQUE_CONTROLLER_PUBLIC
  #define TORQUE_CONTROLLER_LOCAL
#else
  #define TORQUE_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define TORQUE_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define TORQUE_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define TORQUE_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TORQUE_CONTROLLER_PUBLIC
    #define TORQUE_CONTROLLER_LOCAL
  #endif
  #define TORQUE_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // TORQUE_CONTROLLER__VISIBILITY_CONTROL_H_
